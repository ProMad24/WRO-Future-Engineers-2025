#!/usr/bin/env python3

import cv2
import sys
import board
import numpy as np
import time
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import neopixel
from array import array

# Define constants
SERVO_PIN = 23
MOTOR_IN1 = 27
MOTOR_IN2 = 22
BUTTON_PIN = 9
LED_PIN = 10
Jolian = False

OUTPUT = GPIO.OUT
INPUT = GPIO.IN

# Global variables
STOP = False
wall_aligment_state = 0
direction_swap_cycle_threshould = -1

cycle_count = 0
direction = 0
quadrant_count = 0

red_index = 0
green_index = 1

R, G, B = 0, 0, 0

direction_swap_havent_started = True
traffic_index_not_changed_on_cycle_12 = True

parking_near_outer_wall_setup_quadrant = 12
parking_caused_program_override_quadrant_threshould = 13
parking_wall_detected_as_a_wall_quadrant_threshould = 14

# Image variables
raw_frame = np.empty((480, 640, 3), dtype=np.uint8)
frame = np.empty((120, 320, 3), dtype=np.uint8)
hsv = np.empty((120, 320, 3), dtype=np.uint8)

# Traffic light variables
red_mask = np.zeros((120, 320), dtype=np.uint8)
green_mask = np.zeros((120, 320), dtype=np.uint8)

PARALELIPIPED_MIN_AREA = 75

last_detected_traffic_light = -1

target = array('i', [0, 0, 0, 0])  # Biggest traffic light {x, y, area, type}

red_box = []
green_box = []

# Wall variables
left_wall = 0.0
right_wall = 0.0

# Map lines variables
line_cycle_delay = 20
blue_line_threshould = 1100
orange_line_threshould = 1100

blue_line_pixel_count = 0
blue_line_next_allowed_cycle = 0
orange_line_pixel_count = 0
orange_line_next_allowed_cycle = 0

blue_line_detected = False
orange_line_detected = False

blue_line_state = 0
orange_line_state = 0

# P controller variables
kp = 0.30
Err = 0
dir = 0.0

# Parking
PARKING_MIN_AREA = 1000
purple_mask = np.zeros((120, 320), dtype=np.uint8)
parking = array('i', [0, 0, 0])  # parking gate {x, y, area}
purple_box = []

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
def is_button_down():
    return GPIO.input(BUTTON_PIN) == 0
def servo(angle):
    """Adjust and set the servo angle using RPi.GPIO."""

    global SERVO_PIN

    angle += 90
    deviation = 45
    if angle < 90 - deviation:
        angle = 90 - deviation
    if angle > 90 + deviation:
        angle = 90 + deviation
    angle += 5

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
    GPIO.setup(BUTTON_PIN, INPUT)
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
    config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)})
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

def process_hsv(hsv_arr, red_data, green_data, purple_data):
    global blue_line_pixel_count, orange_line_pixel_count, left_wall, right_wall
    blue_line_pixel_count = 0
    orange_line_pixel_count = 0
    left_wall = 0.0
    right_wall = 0.0

    i = 0
    j = 0
    hsv_flat = hsv_arr.reshape(-1)
    
    # Correction: Use the .flat attribute to safely iterate and modify the 2D arrays
    # And make sure to iterate through all 120 * 320 pixels.
    red_data_flat = red_data.flat
    green_data_flat = green_data.flat
    purple_data_flat = purple_data.flat
    
    for p in range(120 * 320):
        red_data_flat[p] = 0
        green_data_flat[p] = 0
        purple_data_flat[p] = 0

        hue_val = int(hsv_flat[i])
        sat = int(hsv_flat[i + 1])
        val = int(hsv_flat[i + 2])

        if sat > 120 and 60 < val < 240:
            if hue_val < 15 or hue_val > 175:
                red_data_flat[p] = 255
        if sat > 120 and 60 < val < 240:   
            if 45 < hue_val < 90:
                green_data_flat[p] = 255

        if sat > 60 and 70 < val < 200:
            if 90 < hue_val < 135:
                blue_line_pixel_count += 1

        if sat > 60 and 125 < val < 240:
            if 0 <= hue_val <= 30:
                orange_line_pixel_count += 1

        if sat > 177 and 93 < val < 255:
            if 130 <= hue_val <= 145:
                purple_data_flat[p] = 255
                if quadrant_count < parking_wall_detected_as_a_wall_quadrant_threshould and val >= 70:
                    if j < 160:
                        left_wall += 1
                    else:
                        right_wall += 1

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
    global blue_line_state, blue_line_detected, blue_line_next_allowed_cycle
    global orange_line_state, orange_line_detected, orange_line_next_allowed_cycle
    global cycle_count

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

def process_traffic_contours(box, type_idx):
    global target
    for contour in box:
        area = cv2.contourArea(contour)
        if area > target[2]:
            boundingBox = cv2.boundingRect(contour)
            if boundingBox[2] < boundingBox[3]:
                moments = cv2.moments(contour)
                if moments['m00'] != 0:
                    x = int(moments['m10'] / moments['m00'])
                    y = int(moments['m01'] / moments['m00'])
                    target = array('i', [x, y, int(area), type_idx])

def process_parking_contours(box):
    global parking
    for contour in box:
        area = cv2.contourArea(contour)
        if area > parking[2]:
            moments = cv2.moments(contour)
            if moments['m00'] != 0:
                x = int(moments['m10'] / moments['m00'])
                y = int(moments['m01'] / moments['m00'])
                parking = array('i', [x, y, int(area)])

def draw(frame, box_red, box_grn):
    for i, contour in enumerate(box_red):
        area = cv2.contourArea(contour)
        if area > PARALELIPIPED_MIN_AREA:
            cv2.drawContours(frame, box_red, i, (0, 0, 255), 2)
            moments = cv2.moments(contour)
            if moments['m00'] != 0:
                center = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            boundingBox = cv2.boundingRect(contour)
            if boundingBox[2] < boundingBox[3]:
                cv2.rectangle(frame, (boundingBox[0], boundingBox[1]),
                             (boundingBox[0] + boundingBox[2], boundingBox[1] + boundingBox[3]),
                             (0, 0, 255), 3)
            else:
                cv2.rectangle(frame, (boundingBox[0], boundingBox[1]),
                             (boundingBox[0] + boundingBox[2], boundingBox[1] + boundingBox[3]),
                             (255, 0, 122), 3)

    for i, contour in enumerate(box_grn):
        area = cv2.contourArea(contour)
        if area > PARALELIPIPED_MIN_AREA:
            cv2.drawContours(frame, box_grn, i, (0, 255, 0), 2)
            moments = cv2.moments(contour)
            if moments['m00'] != 0:
                center = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
                cv2.circle(frame, center, 5, (0, 255, 0), -1)
            boundingBox = cv2.boundingRect(contour)
            if boundingBox[2] < boundingBox[3]:
                cv2.rectangle(frame, (boundingBox[0], boundingBox[1]),
                             (boundingBox[0] + boundingBox[2], boundingBox[1] + boundingBox[3]),
                             (0, 255, 0), 3)
            else:
                cv2.rectangle(frame, (boundingBox[0], boundingBox[1]),
                             (boundingBox[0] + boundingBox[2], boundingBox[1] + boundingBox[3]),
                             (255, 122, 0), 3)

def extra_imagery(hsv_arr):
    blue_mask = np.zeros((120, 320), dtype=np.uint8)
    orange_mask = np.zeros((120, 320), dtype=np.uint8)
    walls = np.zeros((120, 320), dtype=np.uint8)

    hsv_flat = hsv_arr.reshape(-1)
    blue_mask_flat = blue_mask.flat
    orange_mask_flat = orange_mask.flat
    walls_flat = walls.flat

    for p in range(120 * 320):
        i = p * 3
        hue_val = int(hsv_flat[i])
        sat = int(hsv_flat[i + 1])
        val = int(hsv_flat[i + 2])

        if sat > 60 and 70 < val < 200:
            if 90 < hue_val < 135:
                blue_mask_flat[p] = 255

        if sat > 60 and 125 < val < 240:
            if 0 <= hue_val <= 30:
                orange_mask_flat[p] = 255

        if val < 70:
            walls_flat[p] = 255

    cv2.imwrite("blue.png", blue_mask)
    cv2.imwrite("orange.png", orange_mask)
    cv2.imwrite("walls.png", walls)

def cycle(picam2):
    global R, G, B, cycle_count, dir, target, parking, red_box, green_box, purple_box
    global Err, last_detected_traffic_light, quadrant_count, direction
    global red_index, green_index, traffic_index_not_changed_on_cycle_12
    global direction_swap_havent_started, direction_swap_cycle_threshould
    global STOP, wall_aligment_state, raw_frame, frame, hsv

    R, G, B = 122, 122, 122
    cycle_count += 1
    dir = 0.0

    target = array('i', [160, 0, PARALELIPIPED_MIN_AREA, -1])
    parking = array('i', [160, 0, PARKING_MIN_AREA])

    raw = capture_array(picam2)
    raw_frame[:] = raw
    frame = np.empty((120, 320, 3), dtype=np.uint8)
    process_frame(raw_frame, frame)
    hsv_mat = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    process_hsv(hsv_mat, red_mask, green_mask, purple_mask)

    red_box, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    green_box, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    purple_box, _ = cv2.findContours(purple_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    process_traffic_contours(red_box, red_index)
    process_traffic_contours(green_box, green_index)
    process_parking_contours(purple_box)

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

    if target[3] in [1, 2]:  # Green
        Err = -((200 + target[1] * 2.8) - target[0])
        if target[2] > 2000:
            last_detected_traffic_light = 1
    elif target[3] in [0, 3]:  # Red
        Err = (target[0] - (100 - target[1] * 2.8))
        if target[2] > 3000:
            last_detected_traffic_light = 0
    else:
        Err = 0

    if target[3] % 2 == 1:
        R, G, B = 0, 255, 0
    if target[3] % 2 == 0:
        R, G, B = 255, 0, 0
    if blue_line_state != 0:
        R, G, B = 0, 0, 255
    if orange_line_state != 0:
        R, G, B = 255, 122, 0

    dir = Err * kp

    if direction >= 0:
        if left_wall > 0.725:
            dir = 45
        elif right_wall > 0.725:
            dir = -45
    else:
        if right_wall > 0.725:
            dir = -45
        elif left_wall > 0.725:
            dir = 45

    if quadrant_count == 8 and direction_swap_havent_started:
        direction *= -1
        direction_swap_cycle_threshould = cycle_count + 20
        direction_swap_havent_started = False

    if cycle_count < direction_swap_cycle_threshould:
        dir = 45 if direction >= 0 else -45

    if (quadrant_count == parking_near_outer_wall_setup_quadrant and
            traffic_index_not_changed_on_cycle_12 and abs(dir) < 15):
        if direction == 1:
            red_index = 2
        if direction == -1:
            green_index = 3
        traffic_index_not_changed_on_cycle_12 = False
        motor(0.5 * 255)  # Scale back to 0-255 range

    if quadrant_count >= parking_caused_program_override_quadrant_threshould:
        if wall_aligment_state == 0:
            dir = 0
            if left_wall + right_wall > 0.8:
                wall_aligment_state = 1
        else:
            if direction >= 0:
                dir = (left_wall - 0.8) * 50
                if right_wall > 0.5:
                    dir = 45
            else:
                dir = (0.8 - right_wall) * 50
                if left_wall > 0.5:
                    dir = -45

            if parking[2] > 3400:
                STOP = True

            R, G, B = 255, 0, 255

    servo(dir)
    LED_color(R, G, B)

def main():
    global STOP, cycle_count, raw_frame, frame, hsv, red_mask, green_mask, purple_mask
    global servo_pwm, motor_pwm, pixels, picam2

    Setup_GPIO()
    picam2 = Setup_Camera()
    LED_hsv(0, 255, 255)  # Initial blue

    button_sum = 0
    start = time.time()

    cycle(picam2)
    LED_hsv(85, 255, 255)  # Green for start
    motor(45)  # Scale to 0-255 range

    while not STOP:
        cycle(picam2)
        if is_button_down():
            button_sum += 1

    if button_sum < 30:
        motor(0.5 * 255)  # Scale to 0-255 range
        if direction >= 0:
            servo(30)
            time.sleep(0.45)
            servo(-30)
            time.sleep(1.3)
            servo(0)
            time.sleep(0.3)
        else:
            motor(0.5 * 255)
            servo(-30)
            time.sleep(0.45)
            servo(30)
            time.sleep(1.3)
            servo(0)
            time.sleep(0.3)

    motor(0)
    servo(0)
    LED_hsv(0, 255, 255)  # Back to blue

    end = time.time()
    full_time = (end - start) * 1000.0

    print("\n")
    print(f"time         : {full_time / 1000.0:.3f} s")
    print(f"cycle amount : {cycle_count} cycles")
    print(f"speed        : {full_time / cycle_count if cycle_count else 0:.3f} ms / cycle")

    hsv_mat = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    extra_imagery(hsv_mat)
    draw(frame, red_box, green_box)
    cv2.imwrite("input.png", raw_frame)
    cv2.imwrite("frame.png", frame)
    cv2.imwrite("red.png", red_mask)
    cv2.imwrite("green.png", green_mask)
    cv2.imwrite("purple.png", purple_mask)

    button_sum = 0
    while button_sum < 10:
        if is_button_down():
            button_sum += 1

    LED_hsv(0, 0, 0)  # Off
    picam2.close()
    GPIO.cleanup()
    sys.exit(0)
main()
