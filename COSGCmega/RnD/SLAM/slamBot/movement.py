import serial

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

def send_command(cmd):
    ser.write((cmd + "\n").encode())
    ser.flush()

def read_arduino_response():
    response = ""
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        if line:
            print(f"🧾 Arduino says: {line}")
            response += line + " | "
    return response.strip(" | ")

def cleanup_serial():
    ser.close()
