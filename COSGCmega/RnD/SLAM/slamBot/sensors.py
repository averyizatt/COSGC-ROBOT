import os
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def capture_image_and_edges():
    os.system("libcamera-jpeg -o frame.jpg --mode 1640:1232 --width 1280 --height 960 "
              "--shutter 6000 --gain 2 --brightness 0.2 --timeout 1 --quality 90")
    img = cv2.imread("frame.jpg")
    if img is None:
        raise RuntimeError("❌ Failed to load image.")
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 100)
    cv2.imwrite("edges.jpg", edges)
    edge_count = np.count_nonzero(edges)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=50, maxLineGap=10)
    line_count = len(lines) if lines is not None else 0
    return img, edges, edge_count, lines, line_count

def get_distance_cm():
    GPIO.output(TRIG, False)
    time.sleep(0.05)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
    duration = stop_time - start_time
    return round((duration * 34300) / 2, 1)

def cleanup_gpio():
    GPIO.cleanup()
