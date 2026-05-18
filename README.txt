Setup: 
1. Board Identification:
    1.1. If using Arduino nano labelled NANO:
        - In platformio.ini file ensure command is: board = nanoatmega328

    1.2. If using Arduino nano with no label: 
        - In platformio.ini file ensure command is: board = nanoatmega328new

2. COM port setup:
    2.1. Open a terminal (Command Prompt/Powershell/VScode Terminal)
    2.2. Type 'mode' and hit enter
    2.3. Not which 'COM' port is displayed (eg. 'COM7')
        - If unsure, unplug board and re enter the 'mode' command and see which
        board dissapears. or visa versa, type enter 'mode' with it unplugged,
        then plug board in and re enter 'mode' to see which one appears.
    2.4. Change the platformio.ini file to match with your 'COM' port:
        - upload_port = COMX
        - monitor_port = COMX


Information: 
1. This project includes both the arduino embedded code and the interactive game
2. The 'COM' port in the interactive game must be the one that the arduino is using,
    use the methods outlined above for this.
3. Just run the python game file and the arduino will do its setup and be
    ready for the game