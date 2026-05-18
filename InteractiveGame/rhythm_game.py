import pygame
import time
from datetime import datetime
import sys
import os
os.environ['SDL_VIDEODRIVER'] = 'windib'
import serial
import threading
from collections import deque
from pathlib import Path
import csv
import traceback
import re
import random 

# =========================
# Version
# =========================
VERSION_TAG = "csv-summary + FSR v3.0"
print("[BOOT]", VERSION_TAG)

# =========================
# Sensor and session settings
# =========================
SERIAL_PORT = 'COM7'     # Adjust to your environment
BAUDRATE = 115200
TRIGGER_INTERVAL = 0.6   # Seconds: Interval between stimuli (configurable in settings)

# =========================
# EEG output settings (optional)
# =========================
EEG_PORT = 'COM12'
EEG_BAUD = 115200
EEG_ENABLED = True
EEG_RESET_FRAMES = 4
EEG_SEND_IN_TEST = True
EEG_STIM_CODES = [11, 12, 13, 14]
EEG_RESP_CODES = [21, 22, 23, 24]
EEG_RESET_CODE = 0

# =========================
# Vibration motor signal settings (for sending STIM to Arduino)
# =========================
MOTOR_ENABLED        = True          # Enable vibration motor control
MOTOR_USE_SAME_PORT  = True          # True: Use the same Arduino/COM as FSR reception
MOTOR_PORT           = SERIAL_PORT   # Specify a different COM port if False
MOTOR_BAUD           = BAUDRATE

# Handle and lock for the receiving port (same Arduino)
sensor_serial = None
sensor_lock   = threading.Lock()

# Used when operating with a separate port
motor_serial = None
motor_lock   = threading.Lock()

# =========================
# Playback pattern editable
# =========================
PATTERN_STR   = "2,1,3,2,4,1,2,3,4,2"
REPEAT_COUNT  = 50

# =========================
# Keyboard fallback
# =========================
ENABLE_KEYBOARD = True
SENSOR_CONNECTED = False  # Set to True when serial connection is established
KEYBOARD_MAP = {pygame.K_q: 0, pygame.K_w: 1, pygame.K_e: 2, pygame.K_r: 3}

# =========================
# Game result CSV schema (traditional result log)
# =========================
CSV_COLUMNS = [
    "participant","age","block","trial","lane","is_multi_lane",
    "time_difference_ms","early_late","points","feedback","error_type",
    "keys_pressed","correct_keys","num_presses",
    "had_incorrect_press", "first_incorrect_ms"
]


# =========================
# Characteristics of each sensor:
# Left: Unpressed= ~240
# Centre Left: 
# Centre Right: ~250-260
# Right: ~250-260
# =========================

# =========================
# FSR to press detection parameters (tuned for 250 to 400–500 range)
# =========================
FSR_EMA_ALPHA      = 0.02  # Update baseline EMA only when not pressed
FSR_VAL_EMA_ALPHA  = 0.35   # EMA smoothing for raw values

FSR_RISE_ON_DELTAS  = [45, 90, 45, 45] # Per-sensor: base + delta. Sensor 2 (index 1) requires a larger jump.
FSR_RISE_OFF_DELTAS = [35, 70, 35, 35] # Per-sensor: release detection.

FSR_ABS_ON_MINS     = [320, 400, 320, 320] # Per-sensor: absolute minimum for press. Sensor 2 (index 1) has a higher floor.
FSR_ABS_OFF_MAXS    = [350, 450, 350, 350] # Per-sensor: absolute maximum for release.

FSR_DEBOUNCE_MS    = 100    # Debounce time to suppress rapid presses on the same lane

# =========================
# FSR raw data storage (all samples)
# =========================
RAW_HEADER = ["iso_ts","t_perf","sample_idx","fsr1","fsr2","fsr3","fsr4","event","lane","detail"]

# =========================
# Internal state
# =========================
eeg_serial = None
pending_eeg_resets = deque()
global_frame = 0

# FSR raw data for press detection
fsr_baseline  = [None]*4
fsr_val_ema   = [None]*4
fsr_pressed   = [False]*4
fsr_last_time = [0.0]*4
last_fsr_vals = [0,0,0,0]
TIMEOUT_LIMIT_SEC = 1
EARLY_PRESS_THRESHOLD_SEC = 0.1 # Only count presses within this window before stim as "early"
# Raw data storage (written in main thread)
raw_queue = deque()
raw_file = None
raw_writer = None
raw_sample_idx = 0
raw_session_path = None

# Stimulus and response management
stim_last_perf = [None]*4  # Timestamp of stimulus (perf_counter)

# =========================
# EEG helpers
# =========================
def eeg_init():
    global eeg_serial
    if not EEG_ENABLED:
        print("[EEG] disabled")
        return
    try:
        eeg_serial = serial.Serial(
            EEG_PORT, EEG_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0
        )
        print(f"[EEG] opened: {EEG_PORT}")
    except serial.SerialException as e:
        print(f"[EEG] open error on {EEG_PORT} -> {e}")
        eeg_serial = None

def eeg_send(code: int, schedule_reset: bool = True):
    if eeg_serial and eeg_serial.is_open:
        try:
            eeg_serial.write(bytes(chr(code & 0xFF), 'UTF-8'))
            if schedule_reset and EEG_RESET_FRAMES > 0:
                pending_eeg_resets.append(global_frame + EEG_RESET_FRAMES)
        except Exception as e:
            print(f"[EEG] write error -> {e}")

def eeg_tick():
    while pending_eeg_resets and pending_eeg_resets[0] <= global_frame:
        pending_eeg_resets.popleft()
        if eeg_serial and eeg_serial.is_open:
            try:
                eeg_serial.write(bytes(chr(EEG_RESET_CODE), 'UTF-8'))
            except Exception as e:
                print(f"[EEG] reset error -> {e}")

def eeg_close():
    try:
        if eeg_serial and eeg_serial.is_open:
            eeg_serial.write(bytes(chr(EEG_RESET_CODE), 'UTF-8'))
            eeg_serial.close()
            print("[EEG] closed")
    except Exception as e:
        print(f"[EEG] close error -> {e}")

# =========================
# Utilities
# =========================
def ensure_header(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        with path.open("w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(CSV_COLUMNS)

def log_row(csv_path: Path, row: dict):
    ensure_header(csv_path)
    with csv_path.open("a", newline="", encoding="utf-8") as f:
        csv.DictWriter(f, fieldnames=CSV_COLUMNS).writerow(row)

def raw_open_session(data_dir: Path, user_name: str, user_age: str):
    """Open a raw data session file (does nothing if already open)"""
    global raw_file, raw_writer, raw_session_path, raw_sample_idx
    if raw_writer:
        return
    raw_dir = data_dir / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"{(user_name or 'NA')}_{(user_age or 'NA')}_session_all_{ts}.csv"
    raw_session_path = raw_dir / fname
    raw_file = raw_session_path.open("w", newline="", encoding="utf-8")
    raw_writer = csv.writer(raw_file)
    raw_writer.writerow(RAW_HEADER)
    raw_file.flush()
    raw_sample_idx = 0
    print(f"[RAW] logging to {raw_session_path}")

def raw_close_session():
    global raw_file, raw_writer
    try:
        if raw_file:
            raw_file.flush()
            raw_file.close()
            print("[RAW] closed")
    finally:
        raw_file = None
        raw_writer = None

def raw_queue_sample(vals):
    """Queue a sample row for FSR data (called from serial thread)"""
    global raw_sample_idx, last_fsr_vals
    last_fsr_vals = list(vals[:4])
    raw_sample_idx += 1
    raw_queue.append((
        "sample",
        datetime.now().isoformat(),
        time.perf_counter(),
        raw_sample_idx,
        last_fsr_vals
    ))

def raw_queue_event(event: str, lane: int|None, detail: str = ""):
    """Queue an event row (called from main thread)"""
    raw_queue.append((
        "event",
        datetime.now().isoformat(),
        time.perf_counter(),
        -1,
        list(last_fsr_vals),
        event,
        lane if lane is not None else ""
    , detail))

def raw_flush_queue():
    """Write queued data to CSV in the main thread"""
    global raw_writer
    if not raw_writer:
        return
    while raw_queue:
        item = raw_queue.popleft()
        if item[0] == "sample":
            _, iso_ts, tperf, sidx, vals = item
            row = [iso_ts, f"{tperf:.6f}", sidx, vals[0], vals[1], vals[2], vals[3], "", "", ""]
            raw_writer.writerow(row)
        else:
            _, iso_ts, tperf, sidx, vals, ev, lane, detail = item
            row = [iso_ts, f"{tperf:.6f}", sidx, vals[0], vals[1], vals[2], vals[3], ev, lane, detail]
            raw_writer.writerow(row)
    try:
        raw_file.flush()
    except:
        pass

# =========================
# FSR press detection
# =========================
def handle_fsr_sample(vals):
    """
    vals: Array of 4 numerical values (raw FSR values)
    Adds the index of detected rising edge (press) to sensor_queue
    """
    global SENSOR_CONNECTED
    SENSOR_CONNECTED = True
    now = time.perf_counter()

    # Queue raw data
    raw_queue_sample(vals)

    for i, vr in enumerate(vals[:4]):
        # Smooth the value
        if fsr_val_ema[i] is None:
            fsr_val_ema[i] = float(vr)
        else:
            fsr_val_ema[i] = (1.0 - FSR_VAL_EMA_ALPHA) * fsr_val_ema[i] + FSR_VAL_EMA_ALPHA * float(vr)
        v = fsr_val_ema[i]

        # Initialize baseline
        if fsr_baseline[i] is None:
            fsr_baseline[i] = float(v)

        # Update baseline slowly when not pressed
        if not fsr_pressed[i]:
            fsr_baseline[i] = (1.0 - FSR_EMA_ALPHA) * fsr_baseline[i] + FSR_EMA_ALPHA * float(v)

        base = fsr_baseline[i]
        ton  = max(base + FSR_RISE_ON_DELTAS[i], FSR_ABS_ON_MINS[i])
        toff = min(base + FSR_RISE_OFF_DELTAS[i], FSR_ABS_OFF_MAXS[i])
        if toff >= ton - 10:
            toff = ton - 10  # Ensure hysteresis

        if not fsr_pressed[i]:
            # Rising edge + debounce
            if v >= ton and (now - fsr_last_time[i]) * 1000.0 >= FSR_DEBOUNCE_MS:
                fsr_pressed[i]   = True
                fsr_last_time[i] = now
                sensor_queue.append(i)  # Game consumes as usual
        else:
            # During press: Consider released if below threshold
            if v <= toff:
                fsr_pressed[i] = False

# =========================
# Sensor input listener (serial)
# =========================
sensor_queue = deque()

def serial_listener(port=SERIAL_PORT, baudrate=BAUDRATE):
    global SENSOR_CONNECTED, sensor_serial
    try:
        ser = serial.Serial(port, baudrate, timeout=0.05)
        sensor_serial = ser  # Share handle for sending
        SENSOR_CONNECTED = True
        print(f"[INPUT] sensor listening on {port}")
        buff = b""
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            # Example: "FSR:250,260,255,248"
            if line.startswith("FSR:"):
                try:
                    parts = line.replace("FSR:", "").split(",")
                    vals = [int(p.strip()) for p in parts[:4]]
                    if len(vals) == 4:
                        handle_fsr_sample(vals)
                except Exception as e:
                    # Ignore format inconsistencies
                    pass
                continue

            # Backward compatibility: "PRESS:1"
            if line.startswith("PRESS:"):
                try:
                    idx = int(line.split(":")[1]) - 1
                    if 0 <= idx < 4:
                        sensor_queue.append(idx)
                        SENSOR_CONNECTED = True
                except:
                    pass
    except serial.SerialException as e:
        SENSOR_CONNECTED = False
        print("Serial error:", e, "-> Keyboard fallback enabled")

threading.Thread(target=serial_listener, daemon=True).start()
eeg_init()

# =========================
# Resource paths
# =========================
if getattr(sys, 'frozen', False):
    base_path = sys._MEIPASS
else:
    base_path = os.path.dirname(os.path.abspath(__file__))

DATA_DIR = Path(base_path) / "data"
DATA_DIR.mkdir(parents=True, exist_ok=True)

# =========================
# Pygame setup
# =========================

pygame.init()
pygame.mixer.init()
LOGICAL_W, LOGICAL_H = 1280, 720
screen = pygame.display.set_mode((LOGICAL_W, LOGICAL_H), pygame.RESIZABLE)
pygame.display.set_caption(f"Reaction Game ({VERSION_TAG})")
clock = pygame.time.Clock()

trigger_sound = pygame.mixer.Sound(os.path.join(base_path, "resources/sound_fing0_c.wav"))
press_sounds = [
    pygame.mixer.Sound(os.path.join(base_path, "resources/sound_fing1_d.wav")),
    pygame.mixer.Sound(os.path.join(base_path, "resources/sound_fing2_e.wav")),
    pygame.mixer.Sound(os.path.join(base_path, "resources/sound_fing3_f.wav")),
    pygame.mixer.Sound(os.path.join(base_path, "resources/sound_fing4_g.wav")),
]

colors = [(200,100,100),(100,200,100),(100,100,200),(200,200,100)]
trigger_colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0)]

# =========================
# Game state
# =========================
user_name = ""
user_age = ""
input_active = "name"
# State flow: input -> instructions -> confirm -> settings -> pretest_intro -> pretest -> main_intro -> game or test_game -> result -> file_select -> file_loading -> file_summary
state = "input"
pretest_trial_counter = 0
aftertest_trial_counter = 0
column_active = [False]*4
reaction_start = [None]*4  # perf_counter
reaction_display = [""]*4
reaction_display_timer = [0]*4
press_label_timer = [0]*4

# State for tracking incorrect presses within a single trial
trial_incorrect_press_count = [0]*4
trial_first_incorrect_ms = [None]*4
trial_pre_answered = [False]*4 # Flag for anticipatory correct presses

# Viewer state
viewer_files = []
viewer_offset = 0
viewer_selected = None
viewer_error = None
viewer_summary = None

# Summary async loading
loading_summary = False
summary_thread = None
load_error = None

# Test mode
TEST_FLASH_FRAMES = 20
test_flash_timer = [0]*4
# =========================
# Long instructions content (scrollable)
# =========================
LONG_INSTRUCTIONS = """

Adjust the palm support so that your fingers rest comfortably on all of the buttons. Ensure that your hand stays placed in the device for the duration of the trials

Press next when you are comfortable with the device.


""".strip()
# =========================
# Test-intro long instructions content (scrollable)
# =========================
LONG_INSTRUCTIONS_TEST = """
During this trial you will be prompted to press 1 of the 4 finger buttons at a time. When prompted, a visual on the screen will show you which button to press, a sound will play, and you will feel a small vibration under the specified button. Press as fast as you can. 

First, we will give you a practice mode just to get you familiar with pressing the buttons
and what the screen will look like during the trials. There will be no prompts for button presses here, press freely. 

When you are ready, press “Next” to begin practice mode. 

""".strip()
# =========================
# Pre-test long instructions (scrollable)
# =========================
LONG_INSTRUCTIONS_PRETEST = """
The main trials will now begin. 

The trials will consist of 3 main sections. You will start with a randomised sequence of 50 presses, followed by a repetitive sequence of 500 presses, followed by another randomised sequence of 50 presses.

Keep your fingers placed on the buttons gently for the duration of the trial. Try not to lift your hand up when pressing.
 
Press next when ready to commence with the first sequence, 50 random presses.

""".strip()
# =========================
# Main-intro long instructions (scrollable)
# =========================
LONG_INSTRUCTIONS_MAIN = """
Nice job! You have completed the first section.

Hopefully now you are fully comfortable with the device and how the game works.

The next section will be your opportunity to practice and prepare for the final section! It
will be a repetitive sequence of the same 50 button presses. Take some time if needed to rest before the next section, bring up any issues. 

When ready to commence, press “Next”.
""".strip()
# =========================
# After-test intro long instructions (scrollable)
# =========================
LONG_INSTRUCTIONS_AFTERTEST = """
Great job! You are almost done.

Next you will complete the final test. Remember, this final section will be completely
random.

""".strip()

# =========================
# Instructions scroll state
# =========================
instr_surface = None          # rendered tall surface
instr_content_h = 0
instr_scroll = 0
instr_view_rect = None        # viewport rect on screen
instr_next_rect = None        # content-space "Next" button rect
instr_next_rect_sc = None     # screen-space rect (auto-updated each draw)
SCROLL_SPEED = 50             # pixels per wheel/step
# =========================
# Test-intro scroll state
# =========================
test_instr_surface = None
test_instr_content_h = 0
test_instr_scroll = 0
test_instr_view_rect = None
test_instr_next_rect = None
test_instr_next_rect_sc = None
# =========================
# Pre-test intro scroll state
# =========================
pretest_instr_surface = None
pretest_instr_content_h = 0
pretest_instr_scroll = 0
pretest_instr_view_rect = None
pretest_instr_next_rect = None
pretest_instr_next_rect_sc = None
# =========================
# Main-intro scroll state
# =========================
main_instr_surface = None
main_instr_content_h = 0
main_instr_scroll = 0
main_instr_view_rect = None
main_instr_next_rect = None
main_instr_next_rect_sc = None
# =========================
# After-test intro scroll state
# =========================
aftertest_instr_surface = None
aftertest_instr_content_h = 0
aftertest_instr_scroll = 0
aftertest_instr_view_rect = None
aftertest_instr_next_rect = None
aftertest_instr_next_rect_sc = None

# --- 各ブロックの統計 ---
block_stats = {
    "pretest":   {"sum_ms":[0.0]*4, "hits":[0]*4, "incorrect":0, "score":0},
    "main":      {"sum_ms":[0.0]*4, "hits":[0]*4, "incorrect":0, "score":0},
    "aftertest": {"sum_ms":[0.0]*4, "hits":[0]*4, "incorrect":0, "score":0}
}


# Pattern control
sequence = []          # indices 0..3
seq_total = 0
seq_index = 0
last_trigger_time = 0
current_trigger_interval = TRIGGER_INTERVAL
block_no = 1
trial_counter = 0
pending_trials = {}  
IDX_TO_KEY = {0:"v",1:"b",2:"n",3:"m"}
KEYS = ['v','b','n','m']

# =========================
# CSV utilities (viewer)
# =========================
def list_csv_files():
    return sorted(DATA_DIR.glob("*.csv"),
                  key=lambda p: p.stat().st_mtime, reverse=True)
def summarize_csv_with_fallback(path: Path):
    enc_trials = ["utf-8", "utf-8-sig", "cp932", "shift_jis", "latin-1"]
    last_err = None
    for enc in enc_trials:
        try:
            return _summarize_csv_core(path, enc)
        except UnicodeDecodeError as e:
            last_err = e
            continue
        except Exception:
            raise
    if last_err:
        raise last_err
def _summarize_csv_core(path: Path, encoding: str):
    KEYS = ['v','b','n','m']
    sums = {k:0.0 for k in KEYS}     # RT sum for correct trials only
    cnts = {k:0   for k in KEYS}
    lists= {k:[]  for k in KEYS}     # Fallback list that stores all trials
    wrong = 0
    wrong_per_lane = {k:0 for k in KEYS}
    score_sum = 0.0
    score_from_summary = None
    wrong_from_summary = None

    with path.open('r', encoding=encoding, errors="strict", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            lane = (row.get('lane') or '').strip().lower()
            tstr = (row.get('trial') or '').strip()
            err  = (row.get('error_type') or '').strip().lower()

            if tstr.lower() == 'summary_totals':
                try:
                    score_from_summary = float(row.get('points','') or 0.0)
                except:
                    pass
                fb = (row.get('feedback') or '').strip().lower()
                m = re.search(r"incorrect\s*=\s*(\d+)", fb)
                if m:
                    wrong_from_summary = int(m.group(1))
                continue

            if err == 'incorrect_key':
                wrong += 1
                if lane in KEYS:
                    wrong_per_lane[lane] += 1

            is_trial = False
            try:
                int(tstr); is_trial = True
            except:
                pass

            try:
                rt = float(row.get('time_difference_ms', '') or 'nan')
            except:
                rt = float('nan')
            try:
                pts = float(row.get('points', '') or 0.0)
            except:
                pts = 0.0

            if is_trial:
                score_sum += pts

            if lane in KEYS and is_trial and rt == rt:
                lists[lane].append(rt)
                if pts > 0:
                    sums[lane] += rt; cnts[lane] += 1

    avg_ms = {}
    for k in KEYS:
        if cnts[k] > 0:
            avg_ms[k] = sums[k] / cnts[k]
        else:
            avg_ms[k] = (sum(lists[k]) / len(lists[k])) if len(lists[k]) else 0.0

    score = score_from_summary if (score_from_summary is not None) else score_sum
    wrong_total = wrong_from_summary if (wrong_from_summary is not None) else wrong
    return {"score": score, "avg_ms": avg_ms, "wrong": wrong_total, "wrong_per_lane": wrong_per_lane}
def start_loading_summary(p: Path):
    global loading_summary, summary_thread, load_error, viewer_selected, viewer_summary
    viewer_selected = p
    viewer_summary = None
    load_error = None
    loading_summary = True
    print("[SUM] start:", p)

    def worker():
        global viewer_summary, load_error, loading_summary
        try:
            viewer_summary = summarize_csv_with_fallback(p)
            load_error = None
        except Exception:
            load_error = traceback.format_exc()
            viewer_summary = None
        finally:
            loading_summary = False
            print("[SUM] done. error?", bool(load_error))

    summary_thread = threading.Thread(target=worker, daemon=True)
    summary_thread.start()
# =========================
# Drawing helpers
# =========================
def draw_text(text, x, y, center=True, font_size=30, color=(0,0,0)):
    f = pygame.font.SysFont(None, font_size)
    surf = f.render(text, True, color)
    r = surf.get_rect()
    if center: r.center = (x,y)
    else:      r.topleft = (x,y)
    screen.blit(surf, r)
def _wrap_text(text, font, max_w):
    """Simple word-wrap; returns list of lines."""
    lines = []
    for para in text.splitlines():
        para = para.rstrip()
        if not para:
            lines.append("")  # blank line spacing
            continue
        words = para.split()
        cur = ""
        for w in words:
            test = w if not cur else (cur + " " + w)
            if font.size(test)[0] <= max_w:
                cur = test
            else:
                if cur:
                    lines.append(cur)
                cur = w
        if cur:
            lines.append(cur)
    return lines
def _build_instruction_surface_with_text(view_w, body_text, title="Instructions", bottom_button_label="Next"):
    pad = 24
    body_w = view_w - pad*2
    title_fs = 48
    body_fs = 33
    btn_w, btn_h = 260, 64

    f_title = pygame.font.SysFont(None, title_fs)
    f_body  = pygame.font.SysFont(None, body_fs)

    lines = _wrap_text(body_text, f_body, body_w)

    y = pad + 10
    est_h = y + f_title.get_height() + 20 + len(lines)*(f_body.get_height()+6) + 100 + btn_h + pad
    surf = pygame.Surface((view_w, est_h), pygame.SRCALPHA)
    surf.fill((255,255,255))

    # Title
    title_s = f_title.render(title, True, (0,0,120))
    surf.blit(title_s, ((view_w - title_s.get_width())//2, y))
    y += title_s.get_height() + 20

    # Body
    x = pad
    for ln in lines:
        if ln == "":
            y += int(f_body.get_height()*0.6)
            continue
        ln_s = f_body.render(ln, True, (0,0,0))
        surf.blit(ln_s, (x, y))
        y += f_body.get_height() + 6

    y += 40
    # Bottom Next button
    next_rect = pygame.Rect((view_w - btn_w)//2, y, btn_w, btn_h)
    pygame.draw.rect(surf, (0,150,0), next_rect, border_radius=10)
    btn_text = pygame.font.SysFont(None, 30).render(bottom_button_label, True, (255,255,255))
    surf.blit(btn_text, (next_rect.centerx - btn_text.get_width()//2,
                         next_rect.centery - btn_text.get_height()//2))
    y += btn_h + pad

    # Hint
    hint = pygame.font.SysFont(None, 22).render("Scroll to read. The Next button is at the bottom.", True, (90,90,90))
    surf.blit(hint, ((view_w - hint.get_width())//2, y))
    y += hint.get_height() + pad

    return surf, y, next_rect

def _build_instruction_surface(view_w):
    """Render the LONG_INSTRUCTIONS into a tall surface and return (surface, total_h, next_rect)."""
    pad = 24
    body_w = view_w - pad*2
    title_fs = 48
    body_fs = 33
    btn_w, btn_h = 260, 64

    f_title = pygame.font.SysFont(None, title_fs)
    f_body  = pygame.font.SysFont(None, body_fs)

    # Pre-wrap to know height
    lines = _wrap_text(LONG_INSTRUCTIONS, f_body, body_w)

    y = pad + 10
    # Estimate height
    est_h = y + f_title.get_height() + 20 + len(lines)*(f_body.get_height()+6) + 100 + btn_h + pad
    surf = pygame.Surface((view_w, est_h), pygame.SRCALPHA)
    surf.fill((255,255,255))

    # Draw title
    title = "Welcome to our clinical trial!"
    title_s = f_title.render(title, True, (0,0,120))
    surf.blit(title_s, ((view_w - title_s.get_width())//2, y))
    y += title_s.get_height() + 20

    # Draw body
    x = pad
    for ln in lines:
        if ln == "":
            y += int(f_body.get_height()*0.6)
            continue
        ln_s = f_body.render(ln, True, (0,0,0))
        surf.blit(ln_s, (x, y))
        y += f_body.get_height() + 6

    y += 40
    # Draw bottom Next button (content-space)
    next_rect = pygame.Rect((view_w - btn_w)//2, y, btn_w, btn_h)
    pygame.draw.rect(surf, (0,150,0), next_rect, border_radius=10)
    btn_text = pygame.font.SysFont(None, 30).render("Next", True, (255,255,255))
    surf.blit(btn_text, (next_rect.centerx - btn_text.get_width()//2,
                         next_rect.centery - btn_text.get_height()//2))
    y += btn_h + pad

    # Optional “scroll hint” at very bottom
    hint = pygame.font.SysFont(None, 22).render("Scroll to read. The Next button is at the bottom.", True, (90,90,90))
    surf.blit(hint, ((view_w - hint.get_width())//2, y))
    y += hint.get_height() + pad

    return surf, y, next_rect

def draw_aftertest_intro_screen():
    global aftertest_instr_surface, aftertest_instr_content_h, aftertest_instr_view_rect
    global aftertest_instr_next_rect, aftertest_instr_next_rect_sc, aftertest_instr_scroll

    width, height = screen.get_size()
    screen.fill((255,255,255))

    margin_x = 80
    margin_y = 80
    view_w = max(400, width  - margin_x*2)
    view_h = max(300, height - margin_y*2)
    aftertest_instr_view_rect = pygame.Rect(margin_x, margin_y, view_w, view_h)

    # 幅変更時に再生成
    if (not aftertest_instr_surface) or (aftertest_instr_surface.get_width() != view_w):
        aftertest_instr_surface, aftertest_instr_content_h, aftertest_instr_next_rect = _build_instruction_surface_with_text(
            view_w,
            LONG_INSTRUCTIONS_AFTERTEST,
            title="After-Test Instructions",
            bottom_button_label="Next"
        )
        aftertest_instr_scroll = 0

    # スクロール範囲をクランプ
    max_scroll = max(0, aftertest_instr_content_h - view_h)
    if aftertest_instr_scroll < 0: aftertest_instr_scroll = 0
    if aftertest_instr_scroll > max_scroll: aftertest_instr_scroll = max_scroll

    # 可視領域を描画
    area = pygame.Rect(0, aftertest_instr_scroll, view_w, view_h)
    screen.blit(aftertest_instr_surface, aftertest_instr_view_rect.topleft, area)

    # 枠線
    pygame.draw.rect(screen, (0,0,0), aftertest_instr_view_rect, 2)

    # スクロールバー
    if aftertest_instr_content_h > view_h:
        track_w = 10
        track_rect = pygame.Rect(aftertest_instr_view_rect.right + 10, aftertest_instr_view_rect.top, track_w, view_h)
        pygame.draw.rect(screen, (230,230,230), track_rect)
        ratio = view_h / aftertest_instr_content_h
        thumb_h = max(30, int(view_h * ratio))
        thumb_y = track_rect.top if max_scroll == 0 else track_rect.top + int((aftertest_instr_scroll / max_scroll) * (view_h - thumb_h))
        thumb_rect = pygame.Rect(track_rect.left, thumb_y, track_w, thumb_h)
        pygame.draw.rect(screen, (150,150,150), thumb_rect)

    # 画面座標の Next 矩形（既存クリック互換）
    if aftertest_instr_next_rect:
        aftertest_instr_next_rect_sc = aftertest_instr_next_rect.copy()
        aftertest_instr_next_rect_sc.y = aftertest_instr_next_rect.y - aftertest_instr_scroll + aftertest_instr_view_rect.y
        aftertest_instr_next_rect_sc.x = aftertest_instr_next_rect.x + aftertest_instr_view_rect.x
        draw_aftertest_intro_screen.cont_rect = aftertest_instr_next_rect_sc

    # フッターヒント
    hint_f = pygame.font.SysFont(None, 22)
    hs = hint_f.render("Mouse wheel / PgUp/PgDn or ↑/↓ to scroll", True, (90,90,90))
    screen.blit(hs, (aftertest_instr_view_rect.x, aftertest_instr_view_rect.bottom + 12))

    pygame.display.flip()


def measure_text(text, font_size=36):
    f = pygame.font.SysFont(None, font_size)
    return f.size(text)
def draw_axis_ticks_numeric(rect, max_val, steps=4, color=(120,120,120), label_fmt=lambda v: f"{v:.0f}"):
    if max_val <= 0: max_val = 1.0
    for i in range(steps+1):
        frac = i/steps
        y = rect.bottom - int(frac * (rect.height-40)) - 20
        pygame.draw.line(screen, (220,220,220), (rect.x+50, y), (rect.right-10, y), 1)
        draw_text(label_fmt(max_val*frac), rect.x+10, y-10, False, 18, color)
def draw_bar_chart(rect, labels, values, title, y_label, per_bar_colors=None, value_fmt=lambda v:f"{v:.1f}"):
    pygame.draw.rect(screen, (245,245,245), rect); pygame.draw.rect(screen, (0,0,0), rect, 1)
    draw_text(title, rect.centerx, rect.y+18, True, 24)
    draw_text(y_label, rect.x+14, rect.y+42, False, 18, (80,80,80))

    inner = pygame.Rect(rect.x+50, rect.y+50, rect.width-60, rect.height-70)
    max_val = max(values) if values else 1.0
    if max_val <= 0: max_val = 1.0
    draw_axis_ticks_numeric(rect, max_val, steps=4, label_fmt=(lambda v: f"{v:.1f}" if max_val<10 else f"{v:.0f}"))

    n = max(1, len(values))
    gap = 14
    bar_w = max(10, (inner.width - gap*(n+1)) // n)
    for i, v in enumerate(values):
        x = inner.x + gap + i*(bar_w+gap)
        h = 0 if max_val==0 else int((v/max_val)* (inner.height-16))
        y = inner.bottom - h
        col = per_bar_colors[i] if per_bar_colors and i < len(per_bar_colors) else (100,180,255)
        pygame.draw.rect(screen, col, (x, y, bar_w, h))
        draw_text(value_fmt(v), x+bar_w//2, y-12, True, 18, (0,0,0))
        draw_text(labels[i], x+bar_w//2, inner.bottom+6, True, 20, (0,0,0))

# =========================
# Screens
# =========================
def draw_input_screen():
    width, height = screen.get_size()
    screen.fill((255,255,255))
    form_w = min(int(width*0.7), 900)
    x_left = (width - form_w)//2
    label_w = 260
    field_x = x_left + label_w + 20
    field_w = form_w - label_w - 20
    y_name = height//3 - 40
    y_age  = y_name + 120

    draw_text("Enter your Student ID:", x_left, y_name, False)
    draw_text("Enter your age:",  x_left, y_age,  False)

    name_rect = pygame.Rect(field_x, y_name-10, field_w, 50)
    age_rect  = pygame.Rect(field_x, y_age -10, field_w, 50)
    pygame.draw.rect(screen,(0,0,0), name_rect, 2)
    pygame.draw.rect(screen,(0,0,0), age_rect,  2)
    draw_text(user_name, name_rect.x+10, name_rect.y+12, False)
    draw_text(user_age,  age_rect.x+10,  age_rect.y+12,  False)

    cont_rect = pygame.Rect(width//2-100, y_age+120, 200, 54)
    pygame.draw.rect(screen,(0,150,0), cont_rect)
    draw_text("Continue", cont_rect.centerx, cont_rect.centery)

    draw_input_screen.name_rect = name_rect
    draw_input_screen.age_rect  = age_rect
    draw_input_screen.cont_rect = cont_rect

    pygame.display.flip()
def draw_instruction_screen():
    global instr_surface, instr_content_h, instr_view_rect, instr_next_rect, instr_next_rect_sc, instr_scroll
    width, height = screen.get_size()
    screen.fill((255,255,255))

    # Viewport box (nice margins)
    margin_x = 80
    margin_y = 80
    view_w = max(400, width  - margin_x*2)
    view_h = max(300, height - margin_y*2)
    instr_view_rect = pygame.Rect(margin_x, margin_y, view_w, view_h)

    # Build once or when size changed
    if (not instr_surface) or (instr_surface.get_width() != view_w):
        instr_surface, instr_content_h, instr_next_rect = _build_instruction_surface(view_w)
        instr_scroll = 0  # reset scroll when rebuilt

    max_scroll = max(0, instr_content_h - view_h)
    if instr_scroll < 0: instr_scroll = 0
    if instr_scroll > max_scroll: instr_scroll = max_scroll

    # Blit visible slice
    area = pygame.Rect(0, instr_scroll, view_w, view_h)
    screen.blit(instr_surface, instr_view_rect.topleft, area)

    # Draw border
    pygame.draw.rect(screen, (0,0,0), instr_view_rect, 2)

    # Scrollbar (simple)
    if instr_content_h > view_h:
        track_w = 10
        track_rect = pygame.Rect(instr_view_rect.right + 10, instr_view_rect.top, track_w, view_h)
        pygame.draw.rect(screen, (230,230,230), track_rect)
        ratio = view_h / instr_content_h
        thumb_h = max(30, int(view_h * ratio))
        if max_scroll == 0:
            thumb_y = track_rect.top
        else:
            thumb_y = track_rect.top + int((instr_scroll / max_scroll) * (view_h - thumb_h))
        thumb_rect = pygame.Rect(track_rect.left, thumb_y, track_w, thumb_h)
        pygame.draw.rect(screen, (150,150,150), thumb_rect)

    # Export a screen-space rect for the bottom Next button
    if instr_next_rect:
        instr_next_rect_sc = instr_next_rect.copy()
        instr_next_rect_sc.y = instr_next_rect.y - instr_scroll + instr_view_rect.y
        instr_next_rect_sc.x = instr_next_rect.x + instr_view_rect.x
        # For compatibility with existing code path:
        draw_instruction_screen.cont_rect = instr_next_rect_sc

    # Fixed footer hint (optional)
    hint_f = pygame.font.SysFont(None, 22)
    hint = "Mouse wheel / PgUp/PgDn to scroll"
    hs = hint_f.render(hint, True, (90,90,90))
    screen.blit(hs, (instr_view_rect.x, instr_view_rect.bottom + 12))

    pygame.display.flip()


def draw_confirm_screen():
    width, height = screen.get_size()
    screen.fill((255,255,255))
    draw_text("Confirm Information", width//2, height//5)
    draw_text(f"Name: {user_name}", width//2, height//5 + 90)
    draw_text(f"Age:  {user_age}", width//2, height//5 + 140)

    btn_y = height//2 + 40
    gap_x = 240
    start_rect    = pygame.Rect(width//2 - gap_x - 80, btn_y, 160, 50)
    back_rect     = pygame.Rect(width//2 - 80,           btn_y, 160, 50)
    settings_rect = pygame.Rect(width//2 + gap_x - 80,   btn_y, 160, 50)
    test_rect     = pygame.Rect(width//2 - 80, btn_y + 90, 160, 44)
    graphs_rect   = pygame.Rect(width//2 - 80, btn_y + 140, 160, 44)

    pygame.draw.rect(screen,(0,150,0), start_rect);      draw_text("Start",    *start_rect.center)
    pygame.draw.rect(screen,(150,0,0), back_rect);       draw_text("Back",     *back_rect.center)
    pygame.draw.rect(screen,(0,100,200), settings_rect); draw_text("Settings", *settings_rect.center)
    pygame.draw.rect(screen,(200,150,0), test_rect);     draw_text("Test Mode",*test_rect.center)
    pygame.draw.rect(screen,(100,100,100), graphs_rect); draw_text("CSV Result", *graphs_rect.center)

    draw_confirm_screen.start_rect    = start_rect
    draw_confirm_screen.back_rect     = back_rect
    draw_confirm_screen.settings_rect = settings_rect
    draw_confirm_screen.test_rect     = test_rect
    draw_confirm_screen.graphs_rect   = graphs_rect

    pygame.display.flip()

def draw_settings_screen():
    width, height = screen.get_size()
    screen.fill((255,255,255))

    draw_text("Settings", width//2, 90)

    label_fs = 36
    value_fs = 32
    btn_size = 44
    gap = 12
    row1_y = 220
    row2_y = 300

    label_x = width//2 - 260
    group_left = width//2 + 80

    draw_text("Trigger Interval:", label_x, row1_y, center=False, font_size=label_fs)
    value1 = f"{current_trigger_interval:.1f}s"
    value1_w, value1_h = measure_text(value1, value_fs)

    minus_t = pygame.Rect(group_left, row1_y - btn_size//2, btn_size, btn_size)
    value1_x = minus_t.right + gap
    plus_t  = pygame.Rect(value1_x + value1_w + gap, row1_y - btn_size//2, btn_size, btn_size)
    pygame.draw.rect(screen, (0,0,0), minus_t, 1); draw_text("-", minus_t.centerx, minus_t.centery)
    draw_text(value1, value1_x, row1_y - value1_h//2, center=False, font_size=value_fs)
    pygame.draw.rect(screen, (0,0,0), plus_t, 1);  draw_text("+", plus_t.centerx, plus_t.centery)

    draw_text("Repeat Count:", label_x, row2_y, center=False, font_size=label_fs)
    value2 = f"{REPEAT_COUNT}x"
    value2_w, value2_h = measure_text(value2, value_fs)

    minus_r = pygame.Rect(group_left, row2_y - btn_size//2, btn_size, btn_size)
    value2_x = minus_r.right + gap
    plus_r  = pygame.Rect(value2_x + value2_w + gap, row2_y - btn_size//2, btn_size, btn_size)
    pygame.draw.rect(screen, (0,0,0), minus_r, 1); draw_text("-", minus_r.centerx, minus_r.centery)
    draw_text(value2, value2_x, row2_y - value2_h//2, center=False, font_size=value_fs)
    pygame.draw.rect(screen, (0,0,0), plus_r, 1);  draw_text("+", plus_r.centerx, plus_r.centery)

    back_rect = pygame.Rect(width//2 - 90, row2_y + 110, 180, 48)
    pygame.draw.rect(screen, (0,150,0), back_rect); draw_text("Back", *back_rect.center)

    draw_settings_screen.minus_t = minus_t
    draw_settings_screen.plus_t  = plus_t
    draw_settings_screen.minus_r = minus_r
    draw_settings_screen.plus_r  = plus_r
    draw_settings_screen.back    = back_rect

    pygame.display.flip()

def draw_pretest_intro_screen():
    global pretest_instr_surface, pretest_instr_content_h, pretest_instr_view_rect
    global pretest_instr_next_rect, pretest_instr_next_rect_sc, pretest_instr_scroll

    width, height = screen.get_size()
    screen.fill((255,255,255))

    margin_x = 80
    margin_y = 80
    view_w = max(400, width  - margin_x*2)
    view_h = max(300, height - margin_y*2)
    pretest_instr_view_rect = pygame.Rect(margin_x, margin_y, view_w, view_h)

    # 幅が変わったら再ビルド
    if (not pretest_instr_surface) or (pretest_instr_surface.get_width() != view_w):
        pretest_instr_surface, pretest_instr_content_h, pretest_instr_next_rect = _build_instruction_surface_with_text(
            view_w,
            LONG_INSTRUCTIONS_PRETEST,
            title="Pre-Test Instructions",
            bottom_button_label="Next"   # もとの挙動と合わせるなら "Continue" でもOK
        )
        pretest_instr_scroll = 0

    # スクロール範囲
    max_scroll = max(0, pretest_instr_content_h - view_h)
    if pretest_instr_scroll < 0: pretest_instr_scroll = 0
    if pretest_instr_scroll > max_scroll: pretest_instr_scroll = max_scroll

    # 可視領域をブリット
    area = pygame.Rect(0, pretest_instr_scroll, view_w, view_h)
    screen.blit(pretest_instr_surface, pretest_instr_view_rect.topleft, area)

    # 枠
    pygame.draw.rect(screen, (0,0,0), pretest_instr_view_rect, 2)

    # スクロールバー
    if pretest_instr_content_h > view_h:
        track_w = 10
        track_rect = pygame.Rect(pretest_instr_view_rect.right + 10, pretest_instr_view_rect.top, track_w, view_h)
        pygame.draw.rect(screen, (230,230,230), track_rect)
        ratio = view_h / pretest_instr_content_h
        thumb_h = max(30, int(view_h * ratio))
        if max_scroll == 0:
            thumb_y = track_rect.top
        else:
            thumb_y = track_rect.top + int((pretest_instr_scroll / max_scroll) * (view_h - thumb_h))
        thumb_rect = pygame.Rect(track_rect.left, thumb_y, track_w, thumb_h)
        pygame.draw.rect(screen, (150,150,150), thumb_rect)

    # 画面座標の Next/Continue ボタン矩形（既存のクリック判定互換）
    if pretest_instr_next_rect:
        pretest_instr_next_rect_sc = pretest_instr_next_rect.copy()
        pretest_instr_next_rect_sc.y = pretest_instr_next_rect.y - pretest_instr_scroll + pretest_instr_view_rect.y
        pretest_instr_next_rect_sc.x = pretest_instr_next_rect.x + pretest_instr_view_rect.x
        draw_pretest_intro_screen.cont_rect = pretest_instr_next_rect_sc

    # フッターヒント
    hint_f = pygame.font.SysFont(None, 22)
    hs = hint_f.render("Mouse wheel / PgUp/PgDn or ↑/↓ to scroll", True, (90,90,90))
    screen.blit(hs, (pretest_instr_view_rect.x, pretest_instr_view_rect.bottom + 12))

    pygame.display.flip()


def draw_main_intro_screen():
    global main_instr_surface, main_instr_content_h, main_instr_view_rect
    global main_instr_next_rect, main_instr_next_rect_sc, main_instr_scroll

    width, height = screen.get_size()
    screen.fill((255,255,255))

    margin_x = 80
    margin_y = 80
    view_w = max(400, width  - margin_x*2)
    view_h = max(300, height - margin_y*2)
    main_instr_view_rect = pygame.Rect(margin_x, margin_y, view_w, view_h)

    # 幅が変わったら再ビルド
    if (not main_instr_surface) or (main_instr_surface.get_width() != view_w):
        main_instr_surface, main_instr_content_h, main_instr_next_rect = _build_instruction_surface_with_text(
            view_w,
            LONG_INSTRUCTIONS_MAIN,
            title="Main Section Instructions",
            bottom_button_label="Next"
        )
        main_instr_scroll = 0

    # スクロール範囲
    max_scroll = max(0, main_instr_content_h - view_h)
    if main_instr_scroll < 0: main_instr_scroll = 0
    if main_instr_scroll > max_scroll: main_instr_scroll = max_scroll

    # 可視領域を描画
    area = pygame.Rect(0, main_instr_scroll, view_w, view_h)
    screen.blit(main_instr_surface, main_instr_view_rect.topleft, area)

    # 枠
    pygame.draw.rect(screen, (0,0,0), main_instr_view_rect, 2)

    # スクロールバー
    if main_instr_content_h > view_h:
        track_w = 10
        track_rect = pygame.Rect(main_instr_view_rect.right + 10, main_instr_view_rect.top, track_w, view_h)
        pygame.draw.rect(screen, (230,230,230), track_rect)
        ratio = view_h / main_instr_content_h
        thumb_h = max(30, int(view_h * ratio))
        thumb_y = track_rect.top if max_scroll == 0 else track_rect.top + int((main_instr_scroll / max_scroll) * (view_h - thumb_h))
        thumb_rect = pygame.Rect(track_rect.left, thumb_y, track_w, thumb_h)
        pygame.draw.rect(screen, (150,150,150), thumb_rect)

    # 画面座標の Next 矩形（既存クリック互換）
    if main_instr_next_rect:
        main_instr_next_rect_sc = main_instr_next_rect.copy()
        main_instr_next_rect_sc.y = main_instr_next_rect.y - main_instr_scroll + main_instr_view_rect.y
        main_instr_next_rect_sc.x = main_instr_next_rect.x + main_instr_view_rect.x
        draw_main_intro_screen.cont_rect = main_instr_next_rect_sc

    # フッターヒント
    hint_f = pygame.font.SysFont(None, 22)
    hs = hint_f.render("Mouse wheel / PgUp/PgDn or ↑/↓ to scroll", True, (90,90,90))
    screen.blit(hs, (main_instr_view_rect.x, main_instr_view_rect.bottom + 12))

    pygame.display.flip()


def draw_game_screen():
    width, height = screen.get_size()
    cw = width//4
    screen.fill((255,255,255))
    for i in range(4):
        x = i*cw
        col = trigger_colors[i] if column_active[i] else colors[i]
        pygame.draw.rect(screen,col,(x,0,cw,height))
        if reaction_display[i]:
            draw_text(reaction_display[i],x+cw//2,50)
        if press_label_timer[i]>0:
            draw_text("Press",x+cw//2,height//2)
    draw_text(f"Trial {min(seq_index, seq_total)}/{seq_total}", width//2, 20)
    if ENABLE_KEYBOARD and not SENSOR_CONNECTED:
        draw_text("Keyboard mode: Q W E R", width - 170, height - 20, True, 20)
    pygame.display.flip()

def draw_test_game_screen():
    width, height = screen.get_size()
    cw = width//4
    screen.fill((255,255,255))
    for i in range(4):
        x = i*cw
        col = trigger_colors[i] if test_flash_timer[i] > 0 else colors[i]
        pygame.draw.rect(screen,col,(x,0,cw,height))
    draw_text("TEST MODE (Press sensor/QWER)", width//2, 20, True, 28)

    cont_rect = pygame.Rect(width-140,height-60,120,50)
    pygame.draw.rect(screen,(0,150,0),cont_rect); draw_text("Continue",*cont_rect.center)
    draw_test_game_screen.continue_rect = cont_rect

    if ENABLE_KEYBOARD and not SENSOR_CONNECTED:
        draw_text("Keyboard mode: Q W E R", width - 170, height - 20, True, 20)
    pygame.display.flip()


def draw_result_screen():
    width, height = screen.get_size()
    screen.fill((255,255,255))
    draw_text("Session Complete", width//2, 50)

    y = 120
    for block_name, stats in block_stats.items():
        draw_text(block_name.upper(), width//2, y, True, 30, (0,0,120))
        y += 40
        for i in range(4):
            avg = (stats["sum_ms"][i]/stats["hits"][i]) if stats["hits"][i] else 0.0
            draw_text(f"{IDX_TO_KEY[i].upper()} Avg RT: {avg:.1f} ms", width//2, y)
            y += 30
        draw_text(f"Incorrect presses: {stats['incorrect']}", width//2, y)
        y += 30
        draw_text(f"Score: {stats['score']}", width//2, y)
        y += 60

    again_rect = pygame.Rect(330, height-80, 160, 50)
    pygame.draw.rect(screen,(0,100,200),again_rect)
    draw_text("Play Again", *again_rect.center)
    draw_result_screen.again_rect = again_rect
    pygame.display.flip()

def draw_test_intro_screen():
    global test_instr_surface, test_instr_content_h, test_instr_view_rect
    global test_instr_next_rect, test_instr_next_rect_sc, test_instr_scroll

    width, height = screen.get_size()
    screen.fill((255,255,255))

    margin_x = 80
    margin_y = 80
    view_w = max(400, width  - margin_x*2)
    view_h = max(300, height - margin_y*2)
    test_instr_view_rect = pygame.Rect(margin_x, margin_y, view_w, view_h)

    # Build once or when width changed
    if (not test_instr_surface) or (test_instr_surface.get_width() != view_w):
        test_instr_surface, test_instr_content_h, test_instr_next_rect = _build_instruction_surface_with_text(
            view_w,
            LONG_INSTRUCTIONS_TEST,
            title="TEST MODE",
            bottom_button_label="Next"
        )
        test_instr_scroll = 0

    # Clamp scroll
    max_scroll = max(0, test_instr_content_h - view_h)
    if test_instr_scroll < 0: test_instr_scroll = 0
    if test_instr_scroll > max_scroll: test_instr_scroll = max_scroll

    # Blit slice
    area = pygame.Rect(0, test_instr_scroll, view_w, view_h)
    screen.blit(test_instr_surface, test_instr_view_rect.topleft, area)

    # Border
    pygame.draw.rect(screen, (0,0,0), test_instr_view_rect, 2)

    # Scrollbar
    if test_instr_content_h > view_h:
        track_w = 10
        track_rect = pygame.Rect(test_instr_view_rect.right + 10, test_instr_view_rect.top, track_w, view_h)
        pygame.draw.rect(screen, (230,230,230), track_rect)
        ratio = view_h / test_instr_content_h
        thumb_h = max(30, int(view_h * ratio))
        if max_scroll == 0:
            thumb_y = track_rect.top
        else:
            thumb_y = track_rect.top + int((test_instr_scroll / max_scroll) * (view_h - thumb_h))
        thumb_rect = pygame.Rect(track_rect.left, thumb_y, track_w, thumb_h)
        pygame.draw.rect(screen, (150,150,150), thumb_rect)

    # Screen-space Next rect
    if test_instr_next_rect:
        test_instr_next_rect_sc = test_instr_next_rect.copy()
        test_instr_next_rect_sc.y = test_instr_next_rect.y - test_instr_scroll + test_instr_view_rect.y
        test_instr_next_rect_sc.x = test_instr_next_rect.x + test_instr_view_rect.x
        # 互換: 既存コードと同じ属性名にしておく
        draw_test_intro_screen.cont_rect = test_instr_next_rect_sc

    # Footer hint (optional)
    hint_f = pygame.font.SysFont(None, 22)
    hs = hint_f.render("Mouse wheel / PgUp/PgDn or ↑/↓ to scroll", True, (90,90,90))
    screen.blit(hs, (test_instr_view_rect.x, test_instr_view_rect.bottom + 12))

    pygame.display.flip()


def draw_thanks_screen():
    width, height = screen.get_size()
    screen.fill((255,255,255))
    draw_text("Thank you so much! We hope you had an enjoyable experience completing our trial! ", width//2, height//2-40, True, 40, (0,0,120))

    cont_rect = pygame.Rect(width//2-120, height-100, 240, 60)
    pygame.draw.rect(screen,(0,150,0), cont_rect)
    draw_text("Continue", cont_rect.centerx, cont_rect.centery, True, 30, (255,255,255))
    draw_thanks_screen.cont_rect = cont_rect
    pygame.display.flip()


# ----- File select / loading / summary screens -----
def draw_file_select_screen():
    width, height = screen.get_size()
    screen.fill((255,255,255))
    draw_text("Select CSV (data/)", width//2, 40)
    draw_text(f"DATA DIR: {str(DATA_DIR.resolve())}", width//2, 70, True, 20, (80,80,80))

    x0, y0, line_h, max_vis = 100, 120, 36, 12
    pygame.draw.rect(screen, (230,230,230),
                     (x0-20, y0-20, width-2*(x0-20), max_vis*line_h+60), 0)

    if not viewer_files:
        draw_text("No CSV found. Put files into the 'data' folder above.", width//2, y0+40, True, 24, (200,0,0))

    for i in range(max_vis):
        idx = viewer_offset + i
        if idx >= len(viewer_files): break
        p = viewer_files[idx]
        label = f"{p.name}   ({datetime.fromtimestamp(p.stat().st_mtime).strftime('%Y-%m-%d %H:%M')})"
        rect = pygame.Rect(x0, y0+i*line_h, width-2*x0, line_h-6)
        pygame.draw.rect(screen, (255,255,255), rect)
        pygame.draw.rect(screen, (0,0,0), rect, 1)
        draw_text(label, rect.x+8, rect.y+6, center=False, font_size=22)

    prev_rect = pygame.Rect(x0, y0+max_vis*line_h+10, 100, 32)
    next_rect = pygame.Rect(x0+120, y0+max_vis*line_h+10, 100, 32)
    pygame.draw.rect(screen,(0,0,0), prev_rect, 1); draw_text("Prev", prev_rect.centerx, prev_rect.centery)
    pygame.draw.rect(screen,(0,0,0), next_rect, 1); draw_text("Next", next_rect.centerx, next_rect.centery)
    back_rect = pygame.Rect(width-180, height-60, 160, 40)
    pygame.draw.rect(screen,(0,100,200), back_rect); draw_text("Back", *back_rect.center)

    draw_file_select_screen.prev_rect = prev_rect
    draw_file_select_screen.next_rect = next_rect
    draw_file_select_screen.back_rect = back_rect
    draw_file_select_screen.list_area = pygame.Rect(x0, y0, width-2*x0, 12*36)

    pygame.display.flip()
def draw_file_loading_screen():
    width, height = screen.get_size()
    screen.fill((255,255,255))
    title = viewer_selected.name if viewer_selected else "(loading)"
    draw_text("Loading summary...", width//2, height//2 - 40, True, 30, (0,0,0))
    draw_text(title, width//2, height//2 + 10, True, 24, (60,60,60))
    dots = "." * ((global_frame // 20) % 4)
    draw_text(dots, width//2, height//2 + 50, True, 24, (120,120,120))
    pygame.display.flip()
def draw_file_summary_screen():
    width, height = screen.get_size()
    screen.fill((255,255,255))
    title = viewer_selected.name if viewer_selected else "(no file)"
    draw_text(f"Summary: {title}", width//2, 36)
    draw_text(f"DATA DIR: {str(DATA_DIR.resolve())}", width//2, 62, True, 20, (80,80,80))

    if viewer_error:
        lines = viewer_error.strip().splitlines()[-8:]
        y = 110
        draw_text("Error while reading CSV:", width//2, 100, True, 24, (200,0,0))
        for ln in lines:
            draw_text(ln[:120], 40, y, False, 20, (200,0,0)); y += 22
    elif not viewer_summary:
        draw_text("No data", width//2, height//2, True, 28, (200,0,0))
    else:
        info_box = pygame.Rect(60, 86, width-120, 78)
        pygame.draw.rect(screen, (245,245,245), info_box); pygame.draw.rect(screen, (0,0,0), info_box, 1)
        try:
            score_val = viewer_summary['score']
            score_disp = int(score_val) if score_val == int(score_val) else round(score_val,1)
        except:
            score_disp = viewer_summary['score']
        draw_text(f"Total Score: {score_disp}", info_box.x+20, info_box.y+18, False, 26)
        draw_text(f"Wrong (incorrect_key): {viewer_summary['wrong']}", info_box.x+20, info_box.y+46, False, 22, (180,0,0))

        rt_rect = pygame.Rect(60, info_box.bottom+12, width-120, (height-240)//2)
        labels = ['V','B','N','M']; keys = ['v','b','n','m']
        vals_rt = [max(0.0, float(viewer_summary['avg_ms'].get(k,0.0))) for k in keys]
        draw_bar_chart(rt_rect, labels, vals_rt, "Average Reaction Time per finger", "ms",
                       per_bar_colors=trigger_colors, value_fmt=lambda v: f"{v:.1f}")

        miss_rect = pygame.Rect(60, rt_rect.bottom+18, width-120, height - (rt_rect.bottom+18) - 80)
        per_lane = viewer_summary.get('wrong_per_lane', {k:0 for k in keys})
        vals_miss = [int(per_lane.get(k,0)) for k in keys]
        draw_bar_chart(miss_rect, labels, vals_miss, "Misclicks per finger pressed lane", "count",
                       per_bar_colors=trigger_colors, value_fmt=lambda v: f"{int(v)}")

    back_rect = pygame.Rect(60,  height-60, 160, 44)
    pygame.draw.rect(screen,(0,100,200), back_rect);  draw_text("Back", *back_rect.center)

    draw_file_summary_screen.back_rect = back_rect
    pygame.display.flip()

draw_file_summary_screen.back_rect = pygame.Rect(0,0,0,0)

# =========================
# Game helpers
# =========================
def classify_and_score(rt_sec: float):
    if rt_sec <= 0.2: return "Great", 3
    if rt_sec <= 0.5: return "Good",  2
    if rt_sec <= 0.8: return "Late",  1
    return "Late", 1
def build_sequence(pattern_str: str, repeats: int):
    raw = [s.strip() for s in pattern_str.split(",") if s.strip()]
    nums = []
    for s in raw:
        try:
            n = int(s)
            if 1 <= n <= 4:
                nums.append(n-1)
        except:
            pass
    return nums * max(1, int(repeats))

# =========================
# Send vibration motor signals (STIM:n)
# =========================
def motor_send(line: str):
    """
    Send a line to Arduino (appends \n)
    - Same Arduino: Write to sensor_serial
    - Separate Arduino: Write to motor_serial
    """
    if not MOTOR_ENABLED:
        return
    payload = (line.strip() + "\n").encode("ascii", "ignore")
    try:
        if MOTOR_USE_SAME_PORT:
            ser = sensor_serial
            if ser and ser.is_open:
                with sensor_lock:
                    ser.write(payload)
            else:
                # Ignore if receiving port is not initialized (will send once connected)
                pass
        else:
            global motor_serial
            if motor_serial is None or not motor_serial.is_open:
                with motor_lock:
                    motor_serial = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=0)
            with motor_lock:
                motor_serial.write(payload)
    except Exception as e:
        print("[MOTOR] send error:", e)
def trigger_column_idx(idx: int, trial_no: int = None):
    """Activate a stimulus (only one lane active at a time)"""
    if any(column_active): return
    column_active[idx]     = True
    reaction_start[idx]    = time.perf_counter()
    stim_last_perf[idx]    = reaction_start[idx]
    press_label_timer[idx] = 60
    # Reset incorrect press trackers for this new trial
    trial_incorrect_press_count[idx] = 0
    trial_first_incorrect_ms[idx] = None
    trial_pre_answered[idx] = False # Reset pre-answered flag

    motor_send(f"STIM:{idx+1}")
    trigger_sound.play()
    eeg_send(EEG_STIM_CODES[idx])

    # trial_no を渡されたらそれを使う。なければ従来通り trial_counter+1
    trial_label = trial_no if trial_no is not None else trial_counter+1
    raw_queue_event(event="stim", lane=idx, detail=f"trial={trial_label}")
def reset_round(reset_stats=False):
    global sequence, seq_total, seq_index, last_trigger_time, trial_counter, pending_trials
    column_active[:] = [False]*4
    reaction_start[:] = [None]*4
    stim_last_perf[:] = [None]*4
    reaction_display[:] = ["" ]*4
    reaction_display_timer[:] = [0]*4
    press_label_timer[:] = [0]*4
    trial_incorrect_press_count[:] = [0]*4
    trial_first_incorrect_ms[:] = [None]*4
    trial_pre_answered[:] = [False]*4
    trial_counter = 0
    sequence = build_sequence(PATTERN_STR, REPEAT_COUNT)
    seq_total = len(sequence)
    seq_index = 0
    last_trigger_time = time.perf_counter()
    pending_trials = {}

    # 🔑 統計リセットは明示的に reset_stats=True のときだけ
    if reset_stats:
        for blk in block_stats.values():
            blk["sum_ms"] = [0.0]*4
            blk["hits"]   = [0]*4
            blk["incorrect"] = 0
            blk["score"] = 0
def append_summary_rows(csv_path: Path, participant, age, block_name: str):
    stats = block_stats[block_name]
    for i in range(4):
        avg_ms = (stats["sum_ms"][i]/stats["hits"][i]) if stats["hits"][i] else ""
        row = {
            "participant": participant, "age": age, "block": block_name,
            "trial": "summary_avg", "lane": IDX_TO_KEY[i],
            "is_multi_lane": "FALSE",
            "time_difference_ms": round(avg_ms,1) if avg_ms!="" else "",
            "early_late": "avg", "points": "", "feedback": "",
            "error_type": "summary", "keys_pressed": "", "correct_keys": "",
            "num_presses": stats["hits"][i],
            "had_incorrect_press": "", "first_incorrect_ms": ""
        }
        log_row(csv_path, row)
    row_total = {
        "participant": participant, "age": age, "block": block_name,
        "trial": "summary_totals", "lane": "",
        "is_multi_lane": "FALSE",
        "time_difference_ms": "", "early_late": "",
        "points": stats["score"],
        "feedback": f"incorrect={stats['incorrect']}",
        "error_type": "summary_totals", "keys_pressed": "",
        "correct_keys": "", "num_presses": "",
        "had_incorrect_press": "", "first_incorrect_ms": ""
    }
    log_row(csv_path, row_total)


# =========================
# Main loop
# =========================
pygame.event.set_allowed(None)
pygame.event.set_allowed([pygame.QUIT, pygame.KEYDOWN, pygame.MOUSEBUTTONDOWN])

game_running = True
while game_running:
    clock.tick(60)
    global_frame += 1
    eeg_tick()

    # Write raw data (flush thread queue in main thread)
    raw_flush_queue()

    for event in pygame.event.get():
        try:
            if event.type == pygame.QUIT:
                game_running = False

            # Keyboard fallback
            if event.type == pygame.KEYDOWN:
                if state in ("game", "test_game", "pretest","aftertest") and ENABLE_KEYBOARD and (not SENSOR_CONNECTED):
                    if event.key in KEYBOARD_MAP:
                        sensor_queue.append(KEYBOARD_MAP[event.key])

            # Input screen
            if state=="input":
                if event.type==pygame.KEYDOWN:
                    if input_active=="name":
                        if event.key==pygame.K_BACKSPACE: user_name=user_name[:-1]
                        else: user_name+=event.unicode
                    else:
                        if event.key==pygame.K_BACKSPACE: user_age=user_age[:-1]
                        elif event.unicode.isdigit(): user_age+=event.unicode
                if event.type==pygame.MOUSEBUTTONDOWN:
                    x,y=event.pos
                    if getattr(draw_input_screen, "name_rect", pygame.Rect(0,0,0,0)).collidepoint(x,y): input_active="name"
                    elif getattr(draw_input_screen, "age_rect", pygame.Rect(0,0,0,0)).collidepoint(x,y): input_active="age"
                    elif getattr(draw_input_screen, "cont_rect", pygame.Rect(0,0,0,0)).collidepoint(x,y) and user_name and user_age:
                        state="instructions"
                        continue
            elif state == "instructions":
                # Mouse wheel scroll (Pygame sends as MOUSEBUTTONDOWN button 4/5 in your allowed-list setup)
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 4:   # wheel up
                        instr_scroll = max(0, instr_scroll - SCROLL_SPEED)
                    elif event.button == 5: # wheel down
                        instr_scroll = min(max(0, instr_content_h - instr_view_rect.height),
                                        instr_scroll + SCROLL_SPEED)
                    else:
                        # Click "Next" at bottom of content (screen-space rect maintained each draw)
                        x, y = event.pos
                        if hasattr(draw_instruction_screen, "cont_rect") and draw_instruction_screen.cont_rect.collidepoint(x, y):
                            state = "test_intro"
                            continue

                # Keyboard scrolling
                if event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_UP,):
                        instr_scroll = max(0, instr_scroll - SCROLL_SPEED)
                    elif event.key in (pygame.K_DOWN,):
                        instr_scroll = min(max(0, instr_content_h - instr_view_rect.height),
                                        instr_scroll + SCROLL_SPEED)
                    elif event.key in (pygame.K_PAGEUP,):
                        instr_scroll = max(0, instr_scroll - instr_view_rect.height//1.1)
                    elif event.key in (pygame.K_PAGEDOWN,):
                        instr_scroll = min(max(0, instr_content_h - instr_view_rect.height),
                                        instr_scroll + instr_view_rect.height//1.1)
                    elif event.key in (pygame.K_HOME,):
                        instr_scroll = 0
                    elif event.key in (pygame.K_END,):
                        instr_scroll = max(0, instr_content_h - instr_view_rect.height)


            elif state == "test_intro":
                if event.type == pygame.MOUSEBUTTONDOWN:
                    # wheel
                    if event.button == 4:   # up
                        test_instr_scroll = max(0, test_instr_scroll - SCROLL_SPEED)
                    elif event.button == 5: # down
                        test_instr_scroll = min(max(0, test_instr_content_h - test_instr_view_rect.height),
                                                test_instr_scroll + SCROLL_SPEED)
                    else:
                        x, y = event.pos
                        if hasattr(draw_test_intro_screen, "cont_rect") and draw_test_intro_screen.cont_rect.collidepoint(x, y):
                            state = "test_game"
                            continue

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        test_instr_scroll = max(0, test_instr_scroll - SCROLL_SPEED)
                    elif event.key == pygame.K_DOWN:
                        test_instr_scroll = min(max(0, test_instr_content_h - test_instr_view_rect.height),
                                                test_instr_scroll + SCROLL_SPEED)
                    elif event.key == pygame.K_PAGEUP:
                        test_instr_scroll = max(0, test_instr_scroll - test_instr_view_rect.height//1.1)
                    elif event.key == pygame.K_PAGEDOWN:
                        test_instr_scroll = min(max(0, test_instr_content_h - test_instr_view_rect.height),
                                                test_instr_scroll + test_instr_view_rect.height//1.1)
                    elif event.key == pygame.K_HOME:
                        test_instr_scroll = 0
                    elif event.key == pygame.K_END:
                        test_instr_scroll = max(0, test_instr_content_h - test_instr_view_rect.height)


            elif state=="test_game" and event.type==pygame.MOUSEBUTTONDOWN:
                if hasattr(draw_test_game_screen, "continue_rect") and draw_test_game_screen.continue_rect.collidepoint(event.pos):
                    state="pretest_intro"
                    continue

            # Confirm screen
            # Pretest intro screen
            elif state == "pretest_intro":
                if event.type == pygame.MOUSEBUTTONDOWN:
                    # ホイール（4: up, 5: down）
                    if event.button == 4:
                        pretest_instr_scroll = max(0, pretest_instr_scroll - SCROLL_SPEED)
                        continue
                    elif event.button == 5:
                        pretest_instr_scroll = min(max(0, pretest_instr_content_h - pretest_instr_view_rect.height),
                                                pretest_instr_scroll + SCROLL_SPEED)
                        continue
                    # Next/Continue クリック → 既存の開始処理
                    if draw_pretest_intro_screen.cont_rect.collidepoint(event.pos):
                        raw_open_session(DATA_DIR, user_name, user_age)
                        reset_round(reset_stats=True)
                        reset_round(reset_stats=False)  # sequenceだけ再セット
                        sequence = [random.randint(0,3) for _ in range(50)]  # ここはあなたの仕様のまま
                        seq_total = 50
                        seq_index = 0
                        pretest_trial_counter = 0
                        state = "pretest"
                        continue

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        pretest_instr_scroll = max(0, pretest_instr_scroll - SCROLL_SPEED)
                    elif event.key == pygame.K_DOWN:
                        pretest_instr_scroll = min(max(0, pretest_instr_content_h - pretest_instr_view_rect.height),
                                                pretest_instr_scroll + SCROLL_SPEED)
                    elif event.key == pygame.K_PAGEUP:
                        pretest_instr_scroll = max(0, pretest_instr_scroll - pretest_instr_view_rect.height//1.1)
                    elif event.key == pygame.K_PAGEDOWN:
                        pretest_instr_scroll = min(max(0, pretest_instr_content_h - pretest_instr_view_rect.height),
                                                pretest_instr_scroll + pretest_instr_view_rect.height//1.1)
                    elif event.key == pygame.K_HOME:
                        pretest_instr_scroll = 0
                    elif event.key == pygame.K_END:
                        pretest_instr_scroll = max(0, pretest_instr_content_h - pretest_instr_view_rect.height)



            elif state=="confirm" and event.type==pygame.MOUSEBUTTONDOWN:
                x,y=event.pos
                if draw_confirm_screen.start_rect.collidepoint(x,y):
                    raw_open_session(DATA_DIR, user_name, user_age)
                    reset_round(reset_stats=True)
                    state="pretest_intro"
                    continue
            elif state=="result" and event.type==pygame.MOUSEBUTTONDOWN:
                x, y = event.pos
                if draw_result_screen.again_rect.collidepoint(x,y):
                    state = "thanks"
                    continue

                elif draw_confirm_screen.back_rect.collidepoint(x,y):
                    state="input"; continue
                elif draw_confirm_screen.settings_rect.collidepoint(x,y):
                    state="settings"; continue
                elif draw_confirm_screen.test_rect.collidepoint(x,y):
                    state="test_game"; test_flash_timer[:] = [0]*4
                    pygame.event.clear(); draw_test_game_screen(); continue
                elif draw_confirm_screen.graphs_rect.collidepoint(x,y):
                    state = "file_select"
                    viewer_files = list_csv_files()
                    print(f"[VIEW] found {len(viewer_files)} CSV in {DATA_DIR.resolve()}")
                    viewer_offset = 0
                    viewer_selected = None
                    viewer_error = None
                    viewer_summary = None
                    pygame.event.clear(); draw_file_select_screen()
                    continue

            # Settings screen
            elif state=="settings" and event.type==pygame.MOUSEBUTTONDOWN:
                x,y=event.pos
                if draw_settings_screen.minus_t.collidepoint(x,y):
                    current_trigger_interval=max(0.1,round(current_trigger_interval-0.1,2))
                elif draw_settings_screen.plus_t.collidepoint(x,y):
                    current_trigger_interval=min(5.0,round(current_trigger_interval+0.1,2))
                elif draw_settings_screen.minus_r.collidepoint(x,y):
                    REPEAT_COUNT = max(1, REPEAT_COUNT-1)
                elif draw_settings_screen.plus_r.collidepoint(x,y):
                    REPEAT_COUNT = min(99, REPEAT_COUNT+1)
                elif draw_settings_screen.back.collidepoint(x,y):
                    state="confirm"; continue

            elif state == "main_intro":
                if event.type == pygame.MOUSEBUTTONDOWN:
                    # ホイール（4: up, 5: down）
                    if event.button == 4:
                        main_instr_scroll = max(0, main_instr_scroll - SCROLL_SPEED)
                        continue
                    elif event.button == 5:
                        main_instr_scroll = min(max(0, main_instr_content_h - main_instr_view_rect.height),
                                                main_instr_scroll + SCROLL_SPEED)
                        continue
                    # Next クリック → 既存の開始処理
                    if draw_main_intro_screen.cont_rect.collidepoint(event.pos):
                        reset_round(reset_stats=False)  # ← block_stats は消さない（元コード準拠）
                        sequence = build_sequence(PATTERN_STR, REPEAT_COUNT)
                        seq_total = len(sequence); seq_index = 0
                        state = "game"
                        continue

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        main_instr_scroll = max(0, main_instr_scroll - SCROLL_SPEED)
                    elif event.key == pygame.K_DOWN:
                        main_instr_scroll = min(max(0, main_instr_content_h - main_instr_view_rect.height),
                                                main_instr_scroll + SCROLL_SPEED)
                    elif event.key == pygame.K_PAGEUP:
                        main_instr_scroll = max(0, main_instr_scroll - main_instr_view_rect.height//1.1)
                    elif event.key == pygame.K_PAGEDOWN:
                        main_instr_scroll = min(max(0, main_instr_content_h - main_instr_view_rect.height),
                                                main_instr_scroll + main_instr_view_rect.height//1.1)
                    elif event.key == pygame.K_HOME:
                        main_instr_scroll = 0
                    elif event.key == pygame.K_END:
                        main_instr_scroll = max(0, main_instr_content_h - main_instr_view_rect.height)

                
            elif state == "aftertest_intro":
                if event.type == pygame.MOUSEBUTTONDOWN:
                    # ホイール（4: up, 5: down）
                    if event.button == 4:
                        aftertest_instr_scroll = max(0, aftertest_instr_scroll - SCROLL_SPEED)
                        continue
                    elif event.button == 5:
                        aftertest_instr_scroll = min(
                            max(0, aftertest_instr_content_h - aftertest_instr_view_rect.height),
                            aftertest_instr_scroll + SCROLL_SPEED
                        )
                        continue
                    # Next クリック → 既存の aftertest 開始処理
                    if draw_aftertest_intro_screen.cont_rect.collidepoint(event.pos):
                        reset_round(reset_stats=False)  # ← block_stats は消さない（あなたの元コード準拠）
                        sequence = [random.randint(0,3) for _ in range(50)]
                        seq_total = 50
                        seq_index = 0
                        aftertest_trial_counter = 0
                        state = "aftertest"
                        continue

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        aftertest_instr_scroll = max(0, aftertest_instr_scroll - SCROLL_SPEED)
                    elif event.key == pygame.K_DOWN:
                        aftertest_instr_scroll = min(
                            max(0, aftertest_instr_content_h - aftertest_instr_view_rect.height),
                            aftertest_instr_scroll + SCROLL_SPEED
                        )
                    elif event.key == pygame.K_PAGEUP:
                        aftertest_instr_scroll = max(0, aftertest_instr_scroll - aftertest_instr_view_rect.height//1.1)
                    elif event.key == pygame.K_PAGEDOWN:
                        aftertest_instr_scroll = min(
                            max(0, aftertest_instr_content_h - aftertest_instr_view_rect.height),
                            aftertest_instr_scroll + aftertest_instr_view_rect.height//1.1
                        )
                    elif event.key == pygame.K_HOME:
                        aftertest_instr_scroll = 0
                    elif event.key == pygame.K_END:
                        aftertest_instr_scroll = max(0, aftertest_instr_content_h - aftertest_instr_view_rect.height)


            elif state=="thanks" and event.type==pygame.MOUSEBUTTONDOWN:
                if draw_thanks_screen.cont_rect.collidepoint(event.pos):
                    # 名前入力に戻す
                    user_name = ""
                    user_age = ""
                    state = "input"
                    continue
            # Test mode EXIT
            if state == "test_game" and event.type == pygame.MOUSEBUTTONDOWN:
                if not hasattr(draw_test_game_screen, "exit_rect"):
                    draw_test_game_screen(); continue
                x, y = event.pos
                if draw_test_game_screen.exit_rect.collidepoint(x,y):
                    test_flash_timer[:] = [0]*4; state = "confirm"; continue

            # Viewer file select then async load on click
            if state == "file_select" and event.type == pygame.MOUSEBUTTONDOWN:
                if not hasattr(draw_file_select_screen, "back_rect"):
                    draw_file_select_screen(); continue
                x, y = event.pos
                if draw_file_select_screen.back_rect.collidepoint(x,y):
                    state = "confirm"; continue
                elif draw_file_select_screen.prev_rect.collidepoint(x,y):
                    viewer_offset = max(0, viewer_offset-12); draw_file_select_screen(); continue
                elif draw_file_select_screen.next_rect.collidepoint(x,y):
                    viewer_offset = min(max(0, len(viewer_files)-12), viewer_offset+12); draw_file_select_screen(); continue
                else:
                    area = draw_file_select_screen.list_area
                    if area.collidepoint(x,y):
                        idx = (y - area.y)//36
                        real = viewer_offset + int(idx)
                        if 0 <= real < len(viewer_files):
                            candidate = viewer_files[real]
                            if not candidate.exists():
                                print("[VIEW] file vanished:", candidate); draw_file_select_screen(); continue
                            start_loading_summary(candidate)
                            state = "file_loading"
                            pygame.event.clear()
                            draw_file_loading_screen()
                            continue

            # Viewer summary screen back button
            if state == "file_summary":
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if not hasattr(draw_file_summary_screen, "back_rect") or draw_file_summary_screen.back_rect.width == 0:
                        draw_file_summary_screen(); continue
                    x, y = event.pos
                    if draw_file_summary_screen.back_rect.collidepoint(x,y):
                        state = "file_select"; draw_file_select_screen(); continue
                    
        except Exception:
            # Display exceptions in summary screen
            viewer_error = traceback.format_exc()
            state = "file_summary"
            pygame.event.clear()
            draw_file_summary_screen()
            continue

    # ===== State handling =====
    if state=="game":
        now = time.perf_counter()
        outdate = datetime.now().strftime("%Y%m%d")
        csv_path = DATA_DIR / f"{user_name}_{user_age}_{outdate}.csv"
        block_name = "main"

        # ===== 刺激提示（試行番号は提示時に確定）=====
        if (seq_index < seq_total) and (now - last_trigger_time >= current_trigger_interval) and (not any(column_active)):
            stim_idx = sequence[seq_index]

            # ★ pre/after と同じ：ここで試行番号を確定
            trial_counter += 1
            current_trial = trial_counter

            trigger_column_idx(stim_idx, trial_no=current_trial)

            # この試行のメタデータを保存（後で誤入力/正解/タイムアウトに共通で使う）
            pending_trials[current_trial] = {
                "participant": user_name or "NA",
                "age": user_age or "NA",
                "block": block_name,
                "trial": current_trial,
                "lane": IDX_TO_KEY[stim_idx],
                "is_multi_lane": "FALSE",
                "correct_keys": IDX_TO_KEY[stim_idx]
            }

            seq_index += 1
            last_trigger_time = now

        # ===== TIMEOUT 判定（1秒）※センサー処理の前 =====
        for i in range(4):
            if column_active[i] and (reaction_start[i] is not None):
                elapsed = time.perf_counter() - reaction_start[i]
                if elapsed >= TIMEOUT_LIMIT_SEC:
                    current_trial = trial_counter  # 刺激提示時に確定済み
                    dt_ms = round(elapsed * 1000.0, 2)

                    base = pending_trials.get(current_trial, {})
                    row = {
                        **base,
                        "time_difference_ms": dt_ms,
                        "early_late": "Miss",
                        "points": -2,
                        "feedback": "Miss!",
                        "error_type": "timeout",
                        "keys_pressed": "",
                        "num_presses": 0,
                        "had_incorrect_press": "TRUE" if trial_incorrect_press_count[i] > 0 else "FALSE", "first_incorrect_ms": trial_first_incorrect_ms[i]
                    }
                    log_row(csv_path, row)
                    raw_queue_event("timeout", lane=i, detail=f"trial={current_trial};rt_ms={dt_ms}")

                    block_stats[block_name]["incorrect"] += 1
                    block_stats[block_name]["score"] += -2

                    reaction_display[i] = ""
                    reaction_display_timer[i] = 0
                    column_active[i] = False
                    reaction_start[i] = None
                    press_label_timer[i] = 0
                    trial_pre_answered[i] = False # Reset on timeout
                    last_trigger_time = time.perf_counter() # Reset timer on timeout

        # ===== センサー入力処理 =====
        while sensor_queue:
            idx = sensor_queue.popleft()
            active_idx = next((k for k in range(4) if column_active[k]), None)

            # まだ試行が始まっていないなら無視
            if trial_counter <= 0:
                continue
            current_trial = trial_counter

            # 誤レーン or アクティブ無し → ★即終了・確定記録・消灯（pre/after と同じ）
            if active_idx is None:
                # This is a press between trials. Check if it's within the early press window.
                time_until_stim = last_trigger_time + current_trigger_interval - time.perf_counter()

                # Only count as "early" if it's within the defined threshold before the stimulus.
                if 0 < time_until_stim <= EARLY_PRESS_THRESHOLD_SEC:
                    # Check if it matches the UPCOMING stimulus
                    upcoming_stim_idx = sequence[seq_index] if seq_index < seq_total else -1
                    upcoming_stim_idx = sequence[seq_index]

                    # --- ANTICIPATORY CORRECT PRESS ---
                    # ANTICIPATORY CORRECT PRESS
                    if idx == upcoming_stim_idx:
                        # This press is for the upcoming trial. Log it now.
                        trial_counter += 1
                        current_trial = trial_counter
                        dt_ms = round(time_until_stim * -1000.0, 2)
                        rt_sec_abs = abs(dt_ms / 1000.0) # Use positive value for scoring
                        rt_sec_abs = time_until_stim # Use positive value for scoring

                        label, pts = classify_and_score(rt_sec_abs)

                        # Manually create the trial data since pending_trials isn't populated yet
                        base = pending_trials.get(current_trial, {}) # Get upcoming trial data
                        row = {
                            "participant": user_name or "NA", "age": user_age or "NA",
                            "block": block_name, "trial": current_trial,
                            "lane": IDX_TO_KEY[idx], "is_multi_lane": "FALSE",
                            "correct_keys": IDX_TO_KEY[idx],
                            **base,
                            "time_difference_ms": dt_ms, # Log negative RT
                            "early_late": label, "points": pts, "feedback": label,
                            "error_type": "anticipatory_correct",
                            "keys_pressed": IDX_TO_KEY[idx], "num_presses": 1,
                            "had_incorrect_press": "FALSE", "first_incorrect_ms": None
                        }
                        log_row(csv_path, row)
                        raw_queue_event("resp_early_correct", lane=idx, detail=f"trial={current_trial};rt_ms={dt_ms};grade={label}")

                        # Update stats
                        if pts > 0:
                            block_stats[block_name]["sum_ms"][idx] += dt_ms
                            block_stats[block_name]["hits"][idx] += 1
                        block_stats[block_name]["score"] += pts

                        # Set flag to ignore future presses for this trial, but DO NOT advance seq_index yet.
                        # This allows the stimulus to still be presented.
                        trial_pre_answered[idx] = True
                        # Consume the trial, advance sequence, but DO NOT reset last_trigger_time to maintain ISI.
                        seq_index += 1

                    # --- ANTICIPATORY INCORRECT PRESS ---
                    # ANTICIPATORY INCORRECT PRESS
                    else:
                        dt_ms = round(time_until_stim * -1000.0, 2)
                        row = {
                            "participant": user_name or "NA", "age": user_age or "NA",
                            "block": block_name, "trial": f"early_before_{trial_counter + 1}",
                            "lane": IDX_TO_KEY[idx], "is_multi_lane": "FALSE",
                            "time_difference_ms": dt_ms, "early_late": "Early", "points": -2, "feedback": "Early",
                            "time_difference_ms": dt_ms,
                            "early_late": "Early", "points": -2, "feedback": "Early",
                            "error_type": "early_press",
                            "keys_pressed": IDX_TO_KEY[idx], "correct_keys": "", "num_presses": 1,
                            "had_incorrect_press": "FALSE", "first_incorrect_ms": None
                        }
                        log_row(csv_path, row)
                        raw_queue_event("resp_early", lane=idx, detail=f"rt_ms={dt_ms}")

                        block_stats[block_name]["incorrect"] += 1
                        block_stats[block_name]["score"] += -2
                        # Do NOT cancel the upcoming stimulus. The trial will proceed as scheduled.

                # Any press outside this window (too early) is completely ignored.
                continue # Processed the early press, now wait for next event.
            elif idx != active_idx:
                # Note that an incorrect press happened, but do not log a new row.
                rt_sec_incorrect = time.perf_counter() - reaction_start[active_idx]
                trial_incorrect_press_count[active_idx] += 1
                if trial_first_incorrect_ms[active_idx] is None:
                    trial_first_incorrect_ms[active_idx] = round(rt_sec_incorrect * 1000.0, 2)
                block_stats[block_name]["incorrect"] += 1 # Increment total incorrect count
                raw_queue_event("resp_wrong", lane=idx, detail=f"trial={current_trial};correct_lane={active_idx}")
                continue # Move to next input without ending the trial

            # If this trial was pre-answered, ignore all subsequent presses for it.
            if trial_pre_answered[active_idx]:
                continue

            # 正解
            i = active_idx
            rt_sec = time.perf_counter() - reaction_start[i]
            label, pts = classify_and_score(rt_sec)
            if rt_sec > TIMEOUT_LIMIT_SEC:
                label, pts = "Miss", -2
            dt_ms = round(rt_sec * 1000.0, 2)

            base = pending_trials.get(current_trial, {})
            row = {
                **base,
                "time_difference_ms": dt_ms,
                "early_late": label,
                "points": pts,
                "feedback": label,
                "error_type": "",
                "keys_pressed": IDX_TO_KEY[i],
                "num_presses": 1,
                "had_incorrect_press": "TRUE" if trial_incorrect_press_count[i] > 0 else "FALSE",
                "first_incorrect_ms": trial_first_incorrect_ms[i]
            }
            log_row(csv_path, row)
            raw_queue_event("resp", lane=i, detail=f"trial={current_trial};rt_ms={dt_ms};grade={label}")

            if pts > 0:
                block_stats[block_name]["sum_ms"][i] += dt_ms
                block_stats[block_name]["hits"][i] += 1
            else:
                block_stats[block_name]["incorrect"] += 1
            block_stats[block_name]["score"] += pts

            reaction_display[i] = ""
            reaction_display_timer[i] = 0
            column_active[i] = False
            reaction_start[i] = None
            press_label_timer[i] = 0
            trial_pre_answered[i] = False # Reset on correct response
            last_trigger_time = time.perf_counter() # Reset timer on correct response
            try:
                press_sounds[i].play()
            except:
                pass
            eeg_send(EEG_RESP_CODES[i])

        # ===== 終了条件 =====
        if seq_index >= seq_total and not any(column_active):
            state = "aftertest_intro"
            continue

        # ===== 表示タイマー更新 & 描画 =====
        for i in range(4):
            if reaction_display_timer[i] > 0:
                reaction_display_timer[i] -= 1
                if reaction_display_timer[i] == 0:
                    reaction_display[i] = ""
            if press_label_timer[i] > 0:
                press_label_timer[i] -= 1

        draw_game_screen()

    elif state=="test_game":
        while sensor_queue:
            idx = sensor_queue.popleft()
            if 0 <= idx < 4:
                test_flash_timer[idx] = TEST_FLASH_FRAMES
                try: press_sounds[idx].play()
                except: pass
                if EEG_SEND_IN_TEST: eeg_send(EEG_RESP_CODES[idx])
        for i in range(4):
            if test_flash_timer[i] > 0: test_flash_timer[i] -= 1
        draw_test_game_screen()
    
    elif state=="input":
        draw_input_screen()
    
    elif state=="test_intro":
        draw_test_intro_screen()
    
    elif state=="pretest_intro":
        draw_pretest_intro_screen()
    
    elif state=="pretest":
        now = time.perf_counter()
        outdate = datetime.now().strftime("%Y%m%d")
        csv_path = DATA_DIR / f"{user_name}_{user_age}_{outdate}.csv"
        block_name = "pretest"

        # ===== 刺激提示 =====
        if (seq_index < seq_total) and (now - last_trigger_time >= current_trigger_interval) and (not any(column_active)):
            stim_idx = sequence[seq_index]

            pretest_trial_counter += 1
            current_trial = pretest_trial_counter
            trigger_column_idx(stim_idx, trial_no=current_trial)  # v3の拡張

            pending_trials[current_trial] = {
                "participant": user_name or "NA",
                "age": user_age or "NA",
                "block": block_name,
                "trial": current_trial,
                "lane": IDX_TO_KEY[stim_idx],
                "is_multi_lane": "FALSE",
                "correct_keys": IDX_TO_KEY[stim_idx]
            }

            seq_index += 1
            last_trigger_time = now

        # ===== TIMEOUT 判定（1秒）※センサー処理の前 =====
        for i in range(4):
            if column_active[i] and (reaction_start[i] is not None):
                elapsed = time.perf_counter() - reaction_start[i]
                if elapsed >= TIMEOUT_LIMIT_SEC:
                    # 直近の試行番号（現在走っている刺激）
                    current_trial = pretest_trial_counter
                    dt_ms = round(elapsed * 1000.0, 2)

                    base = pending_trials.get(current_trial, {})
                    row = {
                        **base,
                        "time_difference_ms": dt_ms,
                        "early_late": "Miss",
                        "points": -2,
                        "feedback": "Miss!",
                        "error_type": "timeout",
                        "keys_pressed": "",
                        "num_presses": 0,
                        "had_incorrect_press": "TRUE" if trial_incorrect_press_count[i] > 0 else "FALSE", "first_incorrect_ms": trial_first_incorrect_ms[i]
                    }
                    log_row(csv_path, row)
                    raw_queue_event("timeout", lane=i, detail=f"trial={current_trial};rt_ms={dt_ms}")

                    block_stats[block_name]["incorrect"] += 1
                    block_stats[block_name]["score"] += -2

                    reaction_display[i] = ""
                    reaction_display_timer[i] = 0
                    column_active[i] = False
                    reaction_start[i] = None
                    press_label_timer[i] = 0
                    trial_pre_answered[i] = False # Reset on timeout
                    last_trigger_time = time.perf_counter() # Reset timer on timeout

        # ===== センサー入力処理 =====
        while sensor_queue:
            idx = sensor_queue.popleft()
            active_idx = next((k for k in range(4) if column_active[k]), None)

            if pretest_trial_counter <= 0:
                continue
            current_trial = pretest_trial_counter

            if active_idx is None:
                # This is a press between trials. Check if it's within the early press window.
                time_until_stim = last_trigger_time + current_trigger_interval - time.perf_counter()

                # Only count as "early" if it's within the defined threshold before the stimulus.
                if 0 < time_until_stim <= EARLY_PRESS_THRESHOLD_SEC:
                    upcoming_stim_idx = sequence[seq_index]
                    # Check if it matches the UPCOMING stimulus
                    upcoming_stim_idx = sequence[seq_index] if seq_index < seq_total else -1

                    # ANTICIPATORY CORRECT PRESS
                    # --- ANTICIPATORY CORRECT PRESS ---
                    if idx == upcoming_stim_idx:
                        # This press is for the upcoming trial. Log it now.
                        pretest_trial_counter += 1
                        current_trial = pretest_trial_counter
                        dt_ms = round(time_until_stim * -1000.0, 2)
                        rt_sec_abs = time_until_stim
                        rt_sec_abs = abs(dt_ms / 1000.0)

                        label, pts = classify_and_score(rt_sec_abs)
                        base = pending_trials.get(current_trial, {})

                        # Manually create the trial data
                        row = {
                            **base,
                            "participant": user_name or "NA", "age": user_age or "NA",
                            "block": block_name, "trial": current_trial,
                            "lane": IDX_TO_KEY[idx], "is_multi_lane": "FALSE",
                            "correct_keys": IDX_TO_KEY[idx],
                            "time_difference_ms": dt_ms,
                            "early_late": label, "points": pts, "feedback": label,
                            "error_type": "anticipatory_correct",
                            "keys_pressed": IDX_TO_KEY[idx], "num_presses": 1,
                            "had_incorrect_press": "FALSE", "first_incorrect_ms": None
                        }
                        log_row(csv_path, row)
                        raw_queue_event("resp_early_correct", lane=idx, detail=f"trial={current_trial};rt_ms={dt_ms};grade={label}")

                        # Update stats
                        if pts > 0:
                            block_stats[block_name]["sum_ms"][idx] += dt_ms
                            block_stats[block_name]["hits"][idx] += 1
                        block_stats[block_name]["score"] += pts

                        # Consume trial, maintain ISI.
                        seq_index += 1
                        # Set flag to ignore future presses for this trial
                        trial_pre_answered[idx] = True

                    # ANTICIPATORY INCORRECT PRESS
                    # --- ANTICIPATORY INCORRECT PRESS ---
                    else:
                        dt_ms = round(time_until_stim * -1000.0, 2)
                        row = {
                            "participant": user_name or "NA", "age": user_age or "NA",
                            "block": block_name, "trial": f"early_before_{pretest_trial_counter + 1}",
                            "lane": IDX_TO_KEY[idx], "is_multi_lane": "FALSE",
                            "time_difference_ms": dt_ms,
                            "early_late": "Early", "points": -2, "feedback": "Early",
                            "time_difference_ms": dt_ms, "early_late": "Early", "points": -2, "feedback": "Early",
                            "error_type": "early_press",
                            "keys_pressed": IDX_TO_KEY[idx], "correct_keys": "", "num_presses": 1,
                            "had_incorrect_press": "FALSE", "first_incorrect_ms": None
                        }
                        log_row(csv_path, row)
                        raw_queue_event("resp_early", lane=idx, detail=f"rt_ms={dt_ms}")

                        block_stats[block_name]["incorrect"] += 1
                        block_stats[block_name]["score"] += -2

                # Any press outside this window (too early) is completely ignored.
                continue # Processed the early press, now wait for next event.
            elif idx != active_idx:
                # Note that an incorrect press happened, but do not log a new row.
                rt_sec_incorrect = time.perf_counter() - reaction_start[active_idx]
                trial_incorrect_press_count[active_idx] += 1
                if trial_first_incorrect_ms[active_idx] is None:
                    trial_first_incorrect_ms[active_idx] = round(rt_sec_incorrect * 1000.0, 2)
                block_stats[block_name]["incorrect"] += 1 # Increment total incorrect count
                raw_queue_event("resp_wrong", lane=idx, detail=f"trial={current_trial};correct_lane={active_idx}")
                continue # Move to next input without ending the trial

            # If this trial was pre-answered, ignore all subsequent presses for it.
            if trial_pre_answered[active_idx]:
                continue

            # 正解
            i = active_idx
            rt_sec = time.perf_counter() - reaction_start[i]
            label, pts = classify_and_score(rt_sec)
            if rt_sec > TIMEOUT_LIMIT_SEC:
                label, pts = "Miss", -2
            dt_ms = round(rt_sec * 1000.0, 2)

            row = {
                **pending_trials.get(current_trial, {}),
                "time_difference_ms": dt_ms,
                "early_late": label,
                "points": pts,
                "feedback": label,
                "error_type": "",
                "keys_pressed": IDX_TO_KEY[i],
                "num_presses": 1,
                "had_incorrect_press": "TRUE" if trial_incorrect_press_count[i] > 0 else "FALSE",
                "first_incorrect_ms": trial_first_incorrect_ms[i]
            }
            log_row(csv_path, row)
            raw_queue_event("resp", i, f"rt_ms={dt_ms};grade={label}")

            if pts > 0:
                block_stats[block_name]["sum_ms"][i] += dt_ms
                block_stats[block_name]["hits"][i] += 1
            else:
                block_stats[block_name]["incorrect"] += 1
            block_stats[block_name]["score"] += pts

            reaction_display[i] = ""
            reaction_display_timer[i] = 0
            column_active[i] = False
            reaction_start[i] = None
            press_label_timer[i] = 0
            trial_pre_answered[i] = False # Reset on correct response
            last_trigger_time = time.perf_counter() # Reset timer on correct response
            try:
                press_sounds[i].play()
            except:
                pass

        # ===== 表示タイマー更新 =====
        for i in range(4):
            if reaction_display_timer[i] > 0:
                reaction_display_timer[i] -= 1
                if reaction_display_timer[i] == 0:
                    reaction_display[i] = ""
            if press_label_timer[i] > 0:
                press_label_timer[i] -= 1

        # ===== 終了条件 =====
        if seq_index >= seq_total and not any(column_active):
            state = "main_intro"
            continue

        draw_game_screen()
    
    elif state=="aftertest":
        now = time.perf_counter()
        outdate = datetime.now().strftime("%Y%m%d")
        csv_path = DATA_DIR / f"{user_name}_{user_age}_{outdate}.csv"
        block_name = "aftertest"

        # ===== 刺激提示 =====
        if (seq_index < seq_total) and (now - last_trigger_time >= current_trigger_interval) and (not any(column_active)):
            stim_idx = sequence[seq_index]

            aftertest_trial_counter += 1
            current_trial = aftertest_trial_counter
            trigger_column_idx(stim_idx, trial_no=current_trial)

            pending_trials[current_trial] = {
                "participant": user_name or "NA",
                "age": user_age or "NA",
                "block": block_name,
                "trial": current_trial,
                "lane": IDX_TO_KEY[stim_idx],
                "is_multi_lane": "FALSE",
                "correct_keys": IDX_TO_KEY[stim_idx]
            }

            seq_index += 1
            last_trigger_time = now

        # ===== TIMEOUT 判定（1秒）※センサー処理の前 =====
        for i in range(4):
            if column_active[i] and (reaction_start[i] is not None):
                elapsed = time.perf_counter() - reaction_start[i]
                if elapsed >= TIMEOUT_LIMIT_SEC:
                    current_trial = aftertest_trial_counter
                    dt_ms = round(elapsed * 1000.0, 2)

                    base = pending_trials.get(current_trial, {})
                    row = {
                        **base,
                        "time_difference_ms": dt_ms,
                        "early_late": "Miss",
                        "points": -2,
                        "feedback": "Miss!",
                        "error_type": "timeout",
                        "keys_pressed": "",
                        "num_presses": 0,
                        "had_incorrect_press": "TRUE" if trial_incorrect_press_count[i] > 0 else "FALSE", "first_incorrect_ms": trial_first_incorrect_ms[i]
                    }
                    log_row(csv_path, row)
                    raw_queue_event("timeout", lane=i, detail=f"trial={current_trial};rt_ms={dt_ms}")

                    block_stats[block_name]["incorrect"] += 1
                    block_stats[block_name]["score"] += -2

                    reaction_display[i] = ""
                    reaction_display_timer[i] = 0
                    column_active[i] = False
                    reaction_start[i] = None
                    press_label_timer[i] = 0
                    trial_pre_answered[i] = False # Reset on timeout
                    last_trigger_time = time.perf_counter() # Reset timer on timeout

        # ===== センサー入力処理 =====
        while sensor_queue:
            idx = sensor_queue.popleft()
            active_idx = next((k for k in range(4) if column_active[k]), None)

            if aftertest_trial_counter <= 0:
                continue
            current_trial = aftertest_trial_counter

            if active_idx is None:
                # This is a press between trials. Check if it's within the early press window.
                time_until_stim = last_trigger_time + current_trigger_interval - time.perf_counter()

                # Only count as "early" if it's within the defined threshold before the stimulus.
                if 0 < time_until_stim <= EARLY_PRESS_THRESHOLD_SEC:
                    upcoming_stim_idx = sequence[seq_index]
                    # Check if it matches the UPCOMING stimulus
                    upcoming_stim_idx = sequence[seq_index] if seq_index < seq_total else -1

                    # ANTICIPATORY CORRECT PRESS
                    # --- ANTICIPATORY CORRECT PRESS ---
                    if idx == upcoming_stim_idx:
                        # This press is for the upcoming trial. Log it now.
                        aftertest_trial_counter += 1
                        current_trial = aftertest_trial_counter
                        dt_ms = round(time_until_stim * -1000.0, 2)
                        rt_sec_abs = time_until_stim
                        rt_sec_abs = abs(dt_ms / 1000.0)

                        label, pts = classify_and_score(rt_sec_abs)
                        base = pending_trials.get(current_trial, {})

                        # Manually create the trial data
                        row = {
                            **base,
                            "participant": user_name or "NA", "age": user_age or "NA",
                            "block": block_name, "trial": current_trial,
                            "lane": IDX_TO_KEY[idx], "is_multi_lane": "FALSE",
                            "correct_keys": IDX_TO_KEY[idx],
                            "time_difference_ms": dt_ms,
                            "early_late": label, "points": pts, "feedback": label,
                            "error_type": "anticipatory_correct",
                            "keys_pressed": IDX_TO_KEY[idx], "num_presses": 1,
                            "had_incorrect_press": "FALSE", "first_incorrect_ms": None
                        }
                        log_row(csv_path, row)
                        raw_queue_event("resp_early_correct", lane=idx, detail=f"trial={current_trial};rt_ms={dt_ms};grade={label}")

                        # Update stats
                        if pts > 0:
                            block_stats[block_name]["sum_ms"][idx] += dt_ms
                            block_stats[block_name]["hits"][idx] += 1
                        block_stats[block_name]["score"] += pts

                        # Consume trial, maintain ISI.
                        seq_index += 1
                        # Set flag to ignore future presses for this trial
                        trial_pre_answered[idx] = True

                    # ANTICIPATORY INCORRECT PRESS
                    # --- ANTICIPATORY INCORRECT PRESS ---
                    else:
                        dt_ms = round(time_until_stim * -1000.0, 2)
                        row = {
                            "participant": user_name or "NA", "age": user_age or "NA",
                            "block": block_name, "trial": f"early_before_{aftertest_trial_counter + 1}",
                            "lane": IDX_TO_KEY[idx], "is_multi_lane": "FALSE",
                            "time_difference_ms": dt_ms,
                            "early_late": "Early", "points": -2, "feedback": "Early",
                            "time_difference_ms": dt_ms, "early_late": "Early", "points": -2, "feedback": "Early",
                            "error_type": "early_press",
                            "keys_pressed": IDX_TO_KEY[idx], "correct_keys": "", "num_presses": 1,
                            "had_incorrect_press": "FALSE", "first_incorrect_ms": None
                        }
                        log_row(csv_path, row)
                        raw_queue_event("resp_early", lane=idx, detail=f"rt_ms={dt_ms}")

                        block_stats[block_name]["incorrect"] += 1
                        block_stats[block_name]["score"] += -2

                # Any press outside this window (too early) is completely ignored.
                continue # Processed the early press, now wait for next event.
            elif idx != active_idx:
                # Note that an incorrect press happened, but do not log a new row.
                rt_sec_incorrect = time.perf_counter() - reaction_start[active_idx]
                trial_incorrect_press_count[active_idx] += 1
                if trial_first_incorrect_ms[active_idx] is None:
                    trial_first_incorrect_ms[active_idx] = round(rt_sec_incorrect * 1000.0, 2)
                block_stats[block_name]["incorrect"] += 1 # Increment total incorrect count
                raw_queue_event("resp_wrong", lane=idx, detail=f"trial={current_trial};correct_lane={active_idx}")
                continue # Move to next input without ending the trial

            # If this trial was pre-answered, ignore all subsequent presses for it.
            if trial_pre_answered[active_idx]:
                continue

            # 正解
            i = active_idx
            rt_sec = time.perf_counter() - reaction_start[i]
            label, pts = classify_and_score(rt_sec)
            if rt_sec > TIMEOUT_LIMIT_SEC:
                label, pts = "Miss", -2
            dt_ms = round(rt_sec * 1000.0, 2)

            row = {
                **pending_trials.get(current_trial, {}),
                "time_difference_ms": dt_ms,
                "early_late": label,
                "points": pts,
                "feedback": label,
                "error_type": "",
                "keys_pressed": IDX_TO_KEY[i],
                "num_presses": 1,
                "had_incorrect_press": "TRUE" if trial_incorrect_press_count[i] > 0 else "FALSE",
                "first_incorrect_ms": trial_first_incorrect_ms[i]
            }
            log_row(csv_path, row)
            raw_queue_event("resp", i, f"rt_ms={dt_ms};grade={label}")

            if pts > 0:
                block_stats[block_name]["sum_ms"][i] += dt_ms
                block_stats[block_name]["hits"][i] += 1
            else:
                block_stats[block_name]["incorrect"] += 1
            block_stats[block_name]["score"] += pts

            reaction_display[i] = ""
            reaction_display_timer[i] = 0
            column_active[i] = False
            reaction_start[i] = None
            press_label_timer[i] = 0
            trial_pre_answered[i] = False # Reset on correct response
            last_trigger_time = time.perf_counter() # Reset timer on correct response
            try:
                press_sounds[i].play()
            except:
                pass

        # ===== 表示タイマー更新 =====
        for i in range(4):
            if reaction_display_timer[i] > 0:
                reaction_display_timer[i] -= 1
                if reaction_display_timer[i] == 0:
                    reaction_display[i] = ""
            if press_label_timer[i] > 0:
                press_label_timer[i] -= 1

        # ===== 終了条件 =====
        if seq_index >= seq_total and not any(column_active):
            state = "result"
            continue

        draw_game_screen()
    
    elif state=="aftertest_intro":
        draw_aftertest_intro_screen()
    
    elif state=="main_intro":
        draw_main_intro_screen()
    
    elif state=="instructions":
        draw_instruction_screen()
    
    elif state=="confirm":
        draw_confirm_screen()
    
    elif state=="settings":
        draw_settings_screen()
    
    elif state=="result":
        draw_result_screen()
    
    elif state=="thanks":
        draw_thanks_screen()
    
    elif state=="file_select":
        draw_file_select_screen()
    
    elif state=="file_loading":
        draw_file_loading_screen()
        if not loading_summary:
            viewer_error = load_error
            state = "file_summary"
            pygame.event.clear()
            draw_file_summary_screen()
    
    elif state=="file_summary":
        draw_file_summary_screen()
# Shutdown
raw_close_session()
eeg_close()
pygame.quit()