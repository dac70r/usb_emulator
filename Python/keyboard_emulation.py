import serial
import time
import pyautogui

# ============================================================
#                 PACKET FORMAT CONSTANTS
# ============================================================
START_BYTE = 0xAA
END_BYTE   = 0x55

CMD_MOUSE           = 0x01
CMD_KEYBOARD        = 0x02
CMD_MOUSE_KEYBOARD  = 0x03
# ============================================================
#                 PACKET FORMAT CONSTANTS
# ============================================================
HID_KEYCODES = {
    # Letters
    'a': 0x04, 'b': 0x05, 'c': 0x06, 'd': 0x07, 'e': 0x08, 'f': 0x09,
    'g': 0x0A, 'h': 0x0B, 'i': 0x0C, 'j': 0x0D, 'k': 0x0E, 'l': 0x0F,
    'm': 0x10, 'n': 0x11, 'o': 0x12, 'p': 0x13, 'q': 0x14, 'r': 0x15,
    's': 0x16, 't': 0x17, 'u': 0x18, 'v': 0x19, 'w': 0x1A, 'x': 0x1B,
    'y': 0x1C, 'z': 0x1D,

    # Numbers (top row)
    '1': 0x1E, '2': 0x1F, '3': 0x20, '4': 0x21, '5': 0x22,
    '6': 0x23, '7': 0x24, '8': 0x25, '9': 0x26, '0': 0x27,

    # Control keys
    'ENTER':        0x28,
    'ESC':          0x29,
    'BACKSPACE':    0x2A,
    'DELETE':       0x4C,
    'TAB':          0x2B,
    'SPACE':        0x2C,

    # Symbols
    '-': 0x2D,
    '=': 0x2E,
    '[': 0x2F,
    ']': 0x30,
    '\\': 0x31,
    ';': 0x33,
    "'": 0x34,
    '`': 0x35,
    ',': 0x36,
    '.': 0x37,
    '/': 0x38,

    # Arrows
    'RIGHT': 0x4F,
    'LEFT': 0x50,
    'DOWN': 0x51,
    'UP': 0x52,

    # Function keys
    'F1': 0x3A, 'F2': 0x3B, 'F3': 0x3C, 'F4': 0x3D, 'F5': 0x3E,
    'F6': 0x3F, 'F7': 0x40, 'F8': 0x41, 'F9': 0x42, 'F10': 0x43,
    'F11': 0x44, 'F12': 0x45
    
}

# Modifier bit masks for byte 0 of HID report
HID_MODIFIERS = {
    # Modifiers (for Byte 0)
    'LCTRL':  0x01,
    'LSHIFT': 0x02,
    'LALT':   0x04,
    'CTRL':   0x01,
    'SHIFT':  0x02,
    'ALT':    0x04,
    'LGUI':   0x08,
    'RCTRL':  0x10,
    'RSHIFT': 0x20,
    'RALT':   0x40,
    'RGUI':   0x80
}

# Map shifted characters to (base_char, need_shift_bool)
SHIFTED = {
    '!': ('1', True),
    '@': ('2', True),
    '#': ('3', True),
    '$': ('4', True),
    '%': ('5', True),
    '^': ('6', True),
    '&': ('7', True),
    '*': ('8', True),
    '(': ('9', True),
    ')': ('0', True),

    '_': ('-', True),
    '+': ('=', True),
    '{': ('[', True),
    '}': (']', True),
    '|': ('\\', True),
    ':': (';', True),
    '"': ("'", True),
    '~': ('`', True),
    '<': (',', True),
    '>': ('.', True),
    '?': ('/', True),
}

# ============================================================
#                 FRAME ENCODE / DECODE
# ============================================================
def build_frame(cmd: int, data: bytes) -> bytes:
    """Build a protocol frame."""
    length = len(data)
    checksum = (cmd + length + sum(data)) & 0xFF
    return bytes([START_BYTE, cmd, length]) + data + bytes([checksum, END_BYTE])


def parse_frame(frame: bytes):
    """Parse incoming frame, return (cmd, data) or (None, None)."""
    if len(frame) < 5:
        return None, None
    if frame[0] != START_BYTE or frame[-1] != END_BYTE:
        return None, None

    cmd = frame[1]
    length = frame[2]
    data = frame[3:3 + length]
    checksum = frame[3 + length]

    if checksum != (cmd + length + sum(data)) & 0xFF:
        return None, None

    return cmd, data


# ============================================================
#             SERIAL SEND / RECEIVE HELPERS
# ============================================================
def send_frame(ser, cmd, payload: bytes):
    """Build + send a frame."""
    frame = build_frame(cmd, payload)
    ser.write(frame)
    #print("TX →", frame.hex())
    return frame

# Receive 
def receive_frame(ser):
    """Blocking read until a full frame is received."""
    buffer = bytearray()

    while True:
        b = ser.read(1)
        if not b:
            continue

        buffer += b

        if buffer[-1] == END_BYTE:
            cmd, data = parse_frame(buffer)
            #if cmd is None:
                #print("RX Invalid →", buffer.hex())
            #else:
                #print(f"RX ← CMD=0x{cmd:02X}, DATA={data.decode(errors='replace')}")
            return cmd, data

# ============================================================
#                 FRAME ENCODE / DECODE
# ============================================================
def char_to_report(ch):
    # For lower case applications only

    # Letters (uppercase) - ignore 
    if 'A' <= ch <= 'Z':        
        keycode = HID_KEYCODES[ch.lower()]
        return bytes([0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    # Spacebar 
    elif ch == ' ':
        return bytes([0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    # Symbols (!@#$...)
    if ch in SHIFTED:
        base, need_shift = SHIFTED[ch]
        #return HID_KEYCODES[base], need_shift
        return bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    keycode = HID_KEYCODES[ch]
    return bytes([0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00, 0x00])

def get_keycode_with_shift(ch):
    # Letters (lowercase)
    if 'a' <= ch <= 'z':
        return HID_KEYCODES[ch], False

    # Letters (uppercase)
    if 'A' <= ch <= 'Z':
        return HID_KEYCODES[ch.lower()], True

    # Symbols (!@#$...)
    if ch in SHIFTED:
        base, need_shift = SHIFTED[ch]
        return HID_KEYCODES[base], need_shift
    
    # juhao
    if ch == '.':
        return HID_KEYCODES['.'], False 
    
    # Space
    if ch == ' ':
        return HID_KEYCODES['SPACE'], False
    
     # Default: try dictionary lookup
    if ch in HID_KEYCODES:
        return HID_KEYCODES[ch], False

    # Unsupported character
    raise ValueError(f"Unsupported character: {ch!r}")

def string_to_reports(text):
    reports = []
    for ch in text:
        reports.append(char_to_report(ch))      # key down
        #reports.append(bytes(8))               # key up
    return reports

def shifted_string_to_reports(text):
    reports = []
    for ch in text:
        keycode, need_shift = get_keycode_with_shift(ch) 

         # Build HID report
        modifier = 0x02 if need_shift else 0x00
        report = bytes([modifier, 0x00, keycode, 0, 0, 0, 0, 0])

        reports.append(report)                  # key down
        #reports.append(bytes(8))               # key up
    return reports

def function_key_report(key):
    key = key.upper().strip()

    if key not in HID_KEYCODES:
        raise ValueError(f"Unknown key: {key}")

    code = HID_KEYCODES[key]

    # HID report: modifier, reserved, key1…key6
    return bytes([0x00, 0x00, code, 0, 0, 0, 0, 0])

def special_key_report(ch):
    key = ch.upper().strip()

        # 1. Modifier keys (CTRL / SHIFT / ALT / GUI)
    if key in HID_MODIFIERS:
        modifier = HID_MODIFIERS[key]
        return bytes([
            modifier,  # modifier byte
            0x00,      # reserved
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00  # no normal key pressed
        ])
    
    if key not in HID_KEYCODES:
        raise ValueError(f"Unknown key: {key}")

    keycode = HID_KEYCODES[key]
    return bytes([0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00, 0x00])

# # # # # # # # # # # # # # 
#     MOVE MOUSE HELPER FUNCTIONS 
# # # # # # # # # # # # # # 

def chunks(value):
    """Yield HID-safe movement steps (-127 to +127)."""
    while abs(value) > 127:
        yield 127 if value > 0 else -127
        value -= 127 if value > 0 else -127
    yield value

def send_move(dx, dy):
    report = bytes([
        0x00,             # no buttons
        dx & 0xFF,        # two's complement conversion
        dy & 0xFF,
        0x00              # wheel
    ])
    send_frame(ser, CMD_MOUSE, report.hex().encode())

def move_mouse_within_chunks(dx, dy):

    # get current position
    current_x, current_y = pyautogui.position()

    new_x = dx + current_x
    new_y = dy + current_y

    # break into small chunks
    dx_steps = list(chunks(dx))
    dy_steps = list(chunks(dy))

    #print(dx_steps)
    #print(dy_steps)

    # make both lists same length
    max_len = max(len(dx_steps), len(dy_steps))
    dx_steps += [0] * (max_len - len(dx_steps))
    dy_steps += [0] * (max_len - len(dy_steps))

    # send each HID movement
    for sx, sy in zip(dx_steps, dy_steps):
        send_move(sx, sy)
        time.sleep(0.005)  # small delay for realism

def move_mouse_to_absolute(dx, dy):
    
    # get current position
    current_x, current_y = pyautogui.position()
    move_mouse_within_chunks(-current_x, -current_y)

    time.sleep(0.1)
    move_mouse_within_chunks(dx, dy)

# ============================================================
#               REMOTE MODE (ONE-SHOT EXECUTION)
# ============================================================
def mouse_api():
    cmd = CMD_MOUSE

    print("\nSelect Desired Mouse Movement:")
    print("1) Scroll Up")
    print("2) Scroll Down")
    print("3) Simple Left Click")
    print("4) Simple Right Click")
    print("5) Mouse Upwards")
    print("6) Mouse Downwards")
    print("7) Mouse Left")
    print("8) Mouse Right")
    print("9) Mouse to Absolute Position")
    print("10) Custom Mouse Command")
    choice = input("> ").strip()

    # -----------------------------------------
    # Predefined HID packets (4 bytes each)
    # Format: [buttons, X, Y, wheel]
    # -----------------------------------------
    if choice == "1":
        # Scroll Up
        user_text = "00 00 00 FF".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)

    elif choice == "2":
        # Scroll Down
        user_text = "00 00 00 01".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)


    elif choice == "3":
        # Simple Left Click 
        user_text = "01 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.005)
        user_text = "00 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)

    elif choice == "4":
        # Simple Right Click 
        user_text = "02 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.005)
        user_text = "00 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)

    elif choice == "5":
        # Mouse Upwards
        current_x, current_y = pyautogui.position()
        send_move(0, -70)
        #receive_frame(ser)

    elif choice == "6":
        # Scroll Down
        current_x, current_y = pyautogui.position()
        send_move(0, 70)
        #receive_frame(ser)

    elif choice == "7":
        # Simple Left Click 
        current_x, current_y = pyautogui.position()
        send_move(-70, 0)
        #receive_frame(ser)

    elif choice == "8":
        # Simple Right Click 
        current_x, current_y = pyautogui.position()
        send_move(70, 0)
        #receive_frame(ser)

    elif choice == "9":
        # Custom
        current_x, current_y = pyautogui.position()
        print("Current x and y coordinates are: ", current_x, current_y)
        raw = input("\nEnter X and Y with space\n> ")
        # Split input (handles multiple spaces)
        parts = raw.split()

        if len(parts) != 2:
            print("Invalid input! Example: 50 20"   )
            return

        # Convert to integers
        new_x = int(parts[0])
        new_y = int(parts[1])

        move_mouse_to_absolute(new_x, new_y)
        receive_frame(ser)

    elif choice == "10":
        # Custom
        user_text = input("\nEnter 4-byte HID report in hex (e.g. 00 05 00 05):\n> ").replace(" ","").encode("utf-8")

def keyboard_api():
    cmd = CMD_KEYBOARD

    print("\nSelect Keyboard Input Option:")
    print("1) Alpha-numericals")
    print("2) Function Key")
    print("3) Multiple Keys")
    print("4) Arrow Keys")
    print("5) Custom 8-byte HID report")
    choice = input("> ").strip()

    if choice == "1":
        # Alphanumericals
        user_text = input("\nEnter your desired message (space included)> ")
        auto_shift = input("Auto-shift Shift based on characters？(y/n)：").strip().lower().startswith('y')

        if not auto_shift:
            try:
                reports = string_to_reports(user_text)
                print("The length of is: ", len(user_text))
            except ValueError as e:
                print("X Error:", e)
                return

        elif auto_shift:
            try:
                reports = shifted_string_to_reports(user_text)
                print("The length of is: ", len(user_text))
            except ValueError as e:
                print("X Error:", e)
                return

        # FINAL PAYLOAD:  all HID reports concatenated into bytes
        for r in reports:
            hex_string = r.hex()                                # make it text
            user_text = hex_string.encode("utf-8")
            send_frame(ser, cmd, user_text)
            receive_frame(ser)
            time.sleep(0.050)
            

    elif choice == "2":
        key = input("\nEnter function key (F1–F12): ").upper().strip()

        if not (key.startswith("F") and key[1:].isdigit()):
            print("Invalid function key.")
            return

        reports = function_key_report(key)

        send_frame(ser, cmd, reports.hex().encode("utf-8"))
        receive_frame(ser)

    elif choice == "3":
        # Function keys + multi-key support
        raw = input("\nSupport single function key + multiple key/ multiple key. Enter keys separated by commas (e.g. CTRL, ALT, DELETE) or (ALT, F4): ")
        keys = [k.upper().strip() for k in raw.split(",")]

        modifier_byte = 0
        normal_key = []

        for key in keys:
            if key in HID_MODIFIERS:
                modifier_byte = modifier_byte | HID_MODIFIERS[key]
            else:
                try:
                    normal_key.append(HID_KEYCODES[key])   # try exact match
                except KeyError:
                    try:
                        normal_key.append(HID_KEYCODES[key.lower()])  # try lowercase
                    except KeyError:
                        print(f"Unknown key: {key}")
        
        keycode = bytes([modifier_byte, 0x00, normal_key[0], 0x00, 0x00, 0x00, 0x00, 0x00])
        #print(reports)
        send_frame(ser, cmd, keycode.hex().encode("utf-8"))
        #receive_frame(ser)
        #time.sleep(0.100)
            

        # Build HID report for multiple keys

    elif choice == "4":
        # Function keys
        user_text = input("\nEnter your desired message (space included)> ")

    elif choice == "5":
        user_text = input("\nEnter 8-byte HID report in hex (e.g. 00 05 00 05 11 00 12 99):\n> ").replace(" ","").encode("utf-8")
        print("Print text:", user_text)

def mouse_keyboard_api():
    cmd = CMD_MOUSE

    print("\nSelect Keyboard Input Option:")
    print("1) Alpha-numericals")
    print("2) Function Key")
    print("3) Multiple Keys")
    print("4) Arrow Keys")
    print("5) Custom 8-byte HID report")
    choice = input("> ").strip()

    if choice == "1":
        # Alphanumericals
        user_text = input("\nEnter your desired message (space included)> ")
        auto_shift = input("Auto-shift Shift based on characters？(y/n)：").strip().lower().startswith('y')

        if not auto_shift:
            try:
                reports = string_to_reports(user_text)
                print("The length of is: ", len(user_text))
            except ValueError as e:
                print("X Error:", e)
                return

        elif auto_shift:
            try:
                reports = shifted_string_to_reports(user_text)
                print("The length of is: ", len(user_text))
            except ValueError as e:
                print("X Error:", e)
                return
            
        # Simple Left Click 
        user_text = "01 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)
        user_text = "00 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)

        # FINAL PAYLOAD:  all HID reports concatenated into bytes
        cmd = CMD_KEYBOARD
        for r in reports:
            hex_string = r.hex()                                # make it text
            user_text = hex_string.encode("utf-8")
            send_frame(ser, cmd, user_text)
            #receive_frame(ser)
            time.sleep(0.05)
            

    elif choice == "2":
        key = input("\nEnter function key (F1–F12): ").upper().strip()

        if not (key.startswith("F") and key[1:].isdigit()):
            print("Invalid function key.")
            return

        reports = function_key_report(key)

        # Simple Left Click 
        cmd = CMD_MOUSE
        user_text = "01 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)
        user_text = "00 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)

        cmd = CMD_KEYBOARD
        send_frame(ser, cmd, reports.hex().encode("utf-8"))
        receive_frame(ser)

    elif choice == "3":
        # Function keys + multi-key support
        raw = input("\nSupport single function key + multiple key/ multiple key. Enter keys separated by commas (e.g. CTRL, ALT, DELETE) or (ALT, F4): ")
        keys = [k.upper().strip() for k in raw.split(",")]

        modifier_byte = 0
        normal_key = []

        for key in keys:
            if key in HID_MODIFIERS:
                modifier_byte = modifier_byte | HID_MODIFIERS[key]
            else:
                try:
                    normal_key.append(HID_KEYCODES[key])   # try exact match
                except KeyError:
                    try:
                        normal_key.append(HID_KEYCODES[key.lower()])  # try lowercase
                    except KeyError:
                        print(f"Unknown key: {key}")
        
        keycode = bytes([modifier_byte, 0x00, normal_key[0], 0x00, 0x00, 0x00, 0x00, 0x00])
        #print(reports)

        # Simple Left Click 
        cmd = CMD_MOUSE
        user_text = "01 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)
        user_text = "00 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)

        cmd = CMD_KEYBOARD
        send_frame(ser, cmd, keycode.hex().encode("utf-8"))
        #receive_frame(ser)
        #time.sleep(0.100)
            

        # Build HID report for multiple keys

    elif choice == "4":
        # Function keys
        user_text = input("\nEnter your desired message (space included)> ")
        cmd = CMD_MOUSE
        # Simple Left Click 
        user_text = "01 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)
        user_text = "00 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)
        #TODO
        cmd = CMD_KEYBOARD

    elif choice == "5":
        user_text = input("\nEnter 8-byte HID report in hex (e.g. 00 05 00 05 11 00 12 99):\n> ").replace(" ","").encode("utf-8")
        # Simple Left Click 
        cmd = CMD_MOUSE
        user_text = "01 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)
        user_text = "00 00 00 00".replace(" ","").encode("utf-8")
        send_frame(ser, cmd, user_text)
        receive_frame(ser)
        time.sleep(0.05)
        #TODO
        cmd = CMD_KEYBOARD
        print("Print text:", user_text)

# ============================================================
#               REMOTE MODE (ONE-SHOT EXECUTION)
# ============================================================
def remote_mode():

    # ------------------------------
    # Select CMD type
    # ------------------------------
    while True:
        print("\nSelect USB HID Type:   ")
        print("1) Mouse (CMD = 0x01)    ")
        print("2) Keyboard (CMD = 0x02) ")
        print("3) Mouse + Keyboard      ")
        print("4) Exit                  ")

        cmd_choice = input("> ").strip()    

        if cmd_choice == "1":  
            mouse_api()
            continue

        elif cmd_choice == "2":
            keyboard_api()
            continue

        elif cmd_choice == "3":
            mouse_keyboard_api()
            continue

        elif cmd_choice == "4":
            print("Exiting remote mode...")
            break  # <-- exit the while loop

        else:
            print("Invalid choice, please select 1, 2 or 3.")
            continue

# ============================================================
#                      PROGRAM ENTRY POINT
# ============================================================
if __name__ == "__main__":
    # System defaults to LOCAL mode.
    print("\n=== REMOTE MODE SESSION STARTED ===")

    # Open the Serial Terminal, change according to the appropriate terminal used
    ser = serial.Serial('COM33', 115200, timeout=1)

    # Execute main operation
    remote_mode()

    # Close the Serial Terminal
    ser.close()

    print("=== REMOTE MODE SESSION ENDED ===\n")
