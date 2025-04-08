import cv2
import numpy as np
from picamera2 import Picamera2
import time

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(1)

# HSV ranges
orange_lower = np.array([5, 150, 150])
orange_upper = np.array([20, 255, 255])

yellow_lower = np.array([20, 100, 100])
yellow_upper = np.array([35, 255, 255])

blue_lower = np.array([100, 150, 50])
blue_upper = np.array([130, 255, 255])

last_goal = None
cooldown = 0

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Masks
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    # --- Find Yellow Tape ---
    yellow_x = None
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in yellow_contours:
        if cv2.contourArea(c) > 300:
            x, y, w, h = cv2.boundingRect(c)
            yellow_x = x + w // 2
            break  # Use first found

    # --- Find Blue Tape ---
    blue_x = None
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in blue_contours:
        if cv2.contourArea(c) > 300:
            x, y, w, h = cv2.boundingRect(c)
            blue_x = x + w // 2
            break

    # --- Find Orange Ball ---
    ball_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in ball_contours:
        if cv2.contourArea(c) > 300:
            x, y, w, h = cv2.boundingRect(c)
            ball_cx = x + w // 2
            print(f"BALL POSITION: {ball_cx}")

            # --- Check if it's a goal ---
            if cooldown == 0:
                if yellow_x is not None and ball_cx < yellow_x:
                    print("⚽ YELLOW GOAL!")
                    last_goal = "YELLOW"
                    cooldown = 20
                elif blue_x is not None and ball_cx > blue_x:
                    print("⚽ BLUE GOAL!")
                    last_goal = "BLUE"
                    cooldown = 20

    if cooldown > 0:
        cooldown -= 1
