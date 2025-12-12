from flask import Flask, render_template, request, jsonify
import RPi.GPIO as GPIO
import time

app = Flask(__name__, template_folder="templates")

# ---------------- GPIO SETUP ---------------- #
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

AIN1 = 17
AIN2 = 27
BIN1 = 23
BIN2 = 18
STBY = 4
LED = 6

motor_pins = [AIN1, AIN2, BIN1, BIN2, STBY]
for p in motor_pins:
    GPIO.setup(p, GPIO.OUT)
    GPIO.output(p, GPIO.LOW)

GPIO.setup(LED, GPIO.OUT)
GPIO.output(LED, GPIO.LOW)

def motors_on(): GPIO.output(STBY, GPIO.HIGH)
def motors_off(): GPIO.output(STBY, GPIO.LOW)
def stop(): [GPIO.output(p, GPIO.LOW) for p in motor_pins if p != STBY]
def forward(): 
    motors_on()
    GPIO.output(AIN1, GPIO.HIGH); GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.HIGH); GPIO.output(BIN2, GPIO.LOW)
def backward():
    motors_on()
    GPIO.output(AIN1, GPIO.LOW); GPIO.output(AIN2, GPIO.HIGH)
    GPIO.output(BIN1, GPIO.LOW); GPIO.output(BIN2, GPIO.HIGH)
def left():
    motors_on()
    GPIO.output(AIN1, GPIO.LOW); GPIO.output(AIN2, GPIO.HIGH)
    GPIO.output(BIN1, GPIO.HIGH); GPIO.output(BIN2, GPIO.LOW)
def right():
    motors_on()
    GPIO.output(AIN1, GPIO.HIGH); GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.LOW); GPIO.output(BIN2, GPIO.HIGH)

# ---------------- MOTOR ROUTES ---------------- #

@app.route("/")
def home():
    return render_template("control.html")

@app.route("/cmd/<action>")
def command(action):
    if action == "f": forward()
    elif action == "b": backward()
    elif action == "l": left()
    elif action == "r": right()
    elif action == "s": stop()
    return "OK"

@app.route("/led/<state>")
def led(state):
    GPIO.output(LED, GPIO.HIGH if state == "on" else GPIO.LOW)
    return "OK"

# ---------------- PERCEPTION LOGGING ---------------- #

latest_log = {}  # store latest perception + decision

@app.route("/log", methods=["POST"])
def log_update():
    global latest_log
    latest_log = request.json
    return jsonify({"status": "received"})

@app.route("/console")
def show_console():
    return render_template("console.html")

@app.route("/console_data")
def console_data():
    return jsonify(latest_log)

# ---------------- MAIN ---------------- #

if __name__ == "__main__":
    try:
        motors_off()
        app.run(host="0.0.0.0", port=5000)
    finally:
        stop()
        GPIO.cleanup()
