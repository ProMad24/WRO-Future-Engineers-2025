#!/usr/bin/env python3

import cv2
import sys
import board
import numpy as np
import time
from picamera2 import Picamera2, Preview
import RPi.GPIO as GPIO
import neopixel  # Import the neopixel library 
# Define constants
SERVO_PIN = 17
MOTOR_IN1 = 27
MOTOR_IN2 = 22

OUTPUT = GPIO.OUT
INPUT = GPIO.IN
# --------------------------------------------------

STOP = False
mission_end_cycle = int(1e9)
mission_end_not_activated = True

cycle_count = 0
direction = 0
quadrant_count = 0

hue = 0

# --------------------------------------------------
# Image variables

raw_frame = np.empty((480, 640, 3), dtype=np.uint8)
frame = np.empty((120, 320, 3), dtype=np.uint8)
hsv = np.empty((120, 320, 3), dtype=np.uint8)

# --------------------------------------------------
# Wall variables

left_wall = 0.0
right_wall = 0.0

# --------------------------------------------------
# Map lines variables

line_cycle_delay = 10

blue_line_threshould = 1500
orange_line_threshould = 1500

blue_line_pixel_count = 0
blue_line_next_allowed_cycle = 0

orange_line_pixel_count = 0
orange_line_next_allowed_cycle = 0

blue_line_detected = False
orange_line_detected = False

blue_line_state = 0
orange_line_state = 0

# --------------------------------------------------
# P controller variables

kp = 0.25
Err = 0

dir = 0.0

# --------------------------------------------------

def LED_color(r, g, b):
    for i in range(4):
        pixels[i] = (r, g, b)
    pixels.show()
def LED_hsv(hue_val, sat, val):
    #
    # Convert an HSV color to RGB and set the LED accordingly
    r = g = b = 0
    # Normalize hue to the range [0, 360] for full color spectrum
    h = hue_val * 2.0  # Multiply by 2 to scale to [0, 360]
    s = sat / 255.0  # Saturation (1.0 for full saturation)
    v = val / 255.0  # Value (1.0 for full brightness)

    i = int(h / 60.0) % 6  # Find which sector of the color wheel we're in
    f = h / 60.0 - i  # Calculate the fractional part of hue

    p_val = v * (1.0 - s)
    q_val = v * (1.0 - f * s)
    t_val = v * (1.0 - (1.0 - f) * s)

    if i == 0:
        r = v * 255
        g = t_val * 255
        b = p_val * 255
    elif i == 1:
        r = q_val * 255
        g = v * 255
        b = p_val * 255
    elif i == 2:
        r = p_val * 255
        g = v * 255
        b = t_val * 255
    elif i == 3:
        r = p_val * 255
        g = q_val * 255
        b = v * 255
    elif i == 4:
        r = t_val * 255
        g = p_val * 255
        b = v * 255
    elif i == 5:
        r = v * 255
        g = p_val * 255
        b = q_val * 255

    LED_color(int(r), int(g), int(b))
def servo(angle):
    """Adjust and set the servo angle using RPi.GPIO."""

    global SERVO_PIN

    angle += 60
    deviation = 55
    if angle < 60 - deviation:
        angle = 60 - deviation
    if angle > 60 + deviation:
        angle = 60 + deviation
    angle += 0

    min_duty = 2.5  # Duty cycle for 0 degrees
    max_duty = 12.5 # Duty cycle for 180 degrees

    duty_range = max_duty - min_duty
    duty = min_duty + (angle / 180.0) * duty_range

    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.1)
    servo_pwm.ChangeDutyCycle(0)
def motor(speed):  # Speed is -1 to 1
    global MOTOR_IN1, MOTOR_IN2

    speed = max(-100, min(100, speed))
    speed_pwm = abs(speed)  # Calculate absolute speed for PWM

    if speed > 0:  # Forward

        motor1_pwm.ChangeDutyCycle(speed_pwm)
        motor2_pwm.ChangeDutyCycle(0)
    elif speed < 0:  # Reverse
        motor1_pwm.ChangeDutyCycle(0)
        motor2_pwm.ChangeDutyCycle(speed_pwm)
    else:  # Stop
        motor1_pwm.ChangeDutyCycle(0)
        motor2_pwm.ChangeDutyCycle(0)


def Setup_GPIO():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(SERVO_PIN, OUTPUT)
    GPIO.setup(MOTOR_IN1, OUTPUT)
    GPIO.setup(MOTOR_IN2, OUTPUT)
    GPIO.setup(10, OUTPUT)
    global servo_pwm, motor1_pwm, motor2_pwm,pixels
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)
    servo_pwm.start(0)

    motor1_pwm = GPIO.PWM(MOTOR_IN1, 1000)
    motor2_pwm = GPIO.PWM(MOTOR_IN2, 1000)
    motor1_pwm.start(0)
    motor2_pwm.start(0)

    # Initialize NeoPixels
    pixels = neopixel.NeoPixel(board.D10, 4, brightness=0.2, auto_write=False, pixel_order=neopixel.GRB)

def Setup_Camera():
    global picam2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": 'RGB888', "size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    return picam2


def capture_array(picam2):
    raw_frame_arr = picam2.capture_array()
    return raw_frame_arr


def process_frame(raw_frame_arr, frame_arr):
    i = 0
    f = 120 * 320 * 3 - 3

    raw_flat = raw_frame_arr.reshape(-1)
    frame_flat = frame_arr.reshape(-1)

    for y in range(240, 480, 2):
        for x in range(0, 640, 2):
            frame_flat[f] = raw_flat[i]
            frame_flat[f + 1] = raw_flat[i + 1]
            frame_flat[f + 2] = raw_flat[i + 2]
            i += 6
            f -= 3
        i += 640 * 3


def process_hsv(hsv_arr):
    global blue_line_pixel_count, orange_line_pixel_count, left_wall, right_wall
    blue_line_pixel_count = 0
    orange_line_pixel_count = 0

    left_wall = 0.0
    right_wall = 0.0

    i = 0
    j = 0
    total_pixels = 120 * 320
    hsv_flat = hsv_arr.reshape(-1)
    for p in range(total_pixels):
        hue_val = int(hsv_flat[i])
        sat = int(hsv_flat[i + 1])
        val = int(hsv_flat[i + 2])
        if sat > 60 and val > 90 and val < 240:
            if 90 < hue_val < 135:
                blue_line_pixel_count += 1

        if sat > 30 and val > 60 and val < 240:
            if 15 <= hue_val <= 45:
                orange_line_pixel_count += 1

        if val < 70:
            if j < 160:
                left_wall += 1
            else:
                right_wall += 1

        i += 3
        j = (j + 1) % 320

    left_wall /= (160 * 80)
    right_wall /= (160 * 80)


def update_lines():
    global blue_line_pixel_count, blue_line_threshould, blue_line_next_allowed_cycle, blue_line_detected, blue_line_state
    global orange_line_pixel_count, orange_line_threshould, orange_line_next_allowed_cycle, orange_line_detected, orange_line_state
    global cycle_count, line_cycle_delay

    if blue_line_pixel_count > blue_line_threshould:
        blue_line_state = 1
        if cycle_count >= blue_line_next_allowed_cycle:
            blue_line_detected = True
    else:
        blue_line_state = 0
        if blue_line_detected:
            blue_line_state = 2
            blue_line_next_allowed_cycle = cycle_count + line_cycle_delay
        blue_line_detected = False

    if orange_line_pixel_count > orange_line_threshould:
        orange_line_state = 1
        if cycle_count >= orange_line_next_allowed_cycle:
            orange_line_detected = True
    else:
        orange_line_state = 0
        if orange_line_detected:
            orange_line_state = 2
            orange_line_next_allowed_cycle = cycle_count + line_cycle_delay
        orange_line_detected = False


def extra_imagery(hsv_arr):
    walls = np.zeros((120, 320), dtype=np.uint8)
    hsv_flat = hsv_arr.reshape(-1)
    for p in range(120 * 320):
        i = p * 3
        hue_val = int(hsv_flat[i])
        sat = int(hsv_flat[i + 1])
        val = int(hsv_flat[i + 2])
        if val < 70:
            walls.flat[p] = 255
    cv2.imwrite("walls.png", walls)


def cycle(picam2):
    global cycle_count, dir, blue_line_state, orange_line_state, direction, quadrant_count
    global mission_end_not_activated, mission_end_cycle, STOP, hue, left_wall, right_wall
    global blue_line_pixel_count, orange_line_pixel_count, raw_frame, frame, hsv

    cycle_count += 1

    dir = 0.0

    raw = capture_array(picam2)
    raw_frame[:] = raw
    frame = np.empty((120, 320, 3), dtype=np.uint8)
    process_frame(raw_frame, frame)
    hsv_mat = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    process_hsv(hsv_mat)
    update_lines()

    if blue_line_state != 0 and direction == 0:
        direction = -1
    if orange_line_state != 0 and direction == 0:
        direction = 1

    if direction >= 0:
        if orange_line_state == 2:
            quadrant_count += 1
    else:
        if blue_line_state == 2:
            quadrant_count += 1

    color = (122, 0, 0)  # Default color
    if blue_line_state != 0:
        color = (0, 0, 255)  # Blue
    if orange_line_state != 0:
        color = (255, 122, 0)  # Orange
    LED_color(*color)

    if direction == 1:
        dir = (left_wall - 0.5) * 75
    elif direction == -1:
        dir = (0.5 - right_wall) * 75
    else:
        if left_wall > 0.5:
            dir = (left_wall - 0.5) * 75
        if right_wall > 0.5:
            dir = (0.5 - right_wall) * 75

    if quadrant_count == 12 and mission_end_not_activated:
        mission_end_not_activated = False
        mission_end_cycle = cycle_count + 100

    if quadrant_count == 2 :
        STOP = True
    servo(dir)

    # LED_rainbow(hue)

def main():
    global STOP, cycle_count, raw_frame, frame, hsv, hue, picam2, motor1_pwm, motor2_pwm, pixels, servo_pwm
    Setup_GPIO()
    picam2 = Setup_Camera()

    # LED_color(0, 0, 255)  # Initial blue
    LED_hsv(0, 255, 255)

    start = time.time()

    cycle(picam2)
    # LED_color(0, 255, 0)  # Green for start
    LED_hsv(85, 255, 255)

    motor(100)
    while not STOP:
        cycle(picam2)
        print(STOP)
    motor(0)
    servo(0)

    # LED_color(0, 0, 255)  # Back to blue
    LED_hsv(0, 255, 255)

    end = time.time()
    duration = end - start
    full_time = duration * 1000.0

    print("\n")
    print("time         : {:.3f} s".format(full_time / 1000.0))
    print("cycle amount : {} cycles".format(cycle_count))
    print("speed        : {:.3f} ms / cycle".format(full_time / cycle_count if cycle_count else 0))

    # Save extra imagery outputs
    hsv_mat = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    extra_imagery(hsv_mat)
    cv2.imwrite("input.png", raw_frame)
    cv2.imwrite("frame.png", frame)

    sys.exit(0)
main()
