from flask import Flask, render_template, request, jsonify
import time

from motor_control import MotorController

app = Flask(__name__, template_folder="templates")

# Motor controller (abstracted)
motor = MotorController()

# LED pin handled by MotorController simulation or the GPIO layer in that module if needed

# ---------------- MOTOR ROUTES ---------------- #

@app.route("/")
def home():
    return render_template("control.html")

@app.route("/cmd/<action>")
def command(action):
    # Map simple single-letter commands to MotorController API
    if action == "f":
        motor.forward()
    elif action == "b":
        motor.reverse()
    elif action == "l":
        motor.turn_left()
    elif action == "r":
        motor.turn_right()
    elif action == "s":
        motor.stop()
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
        motor.stop()
        app.run(host="0.0.0.0", port=5000)
    finally:
        motor.cleanup()
