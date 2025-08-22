# Obstacle Challenge:
The solution implemented for this challenge integrates a color-based navigation system to complete the laps around the mat and execute the parking process. The robot utilizes its Pi Camera to detect red and green pillars, which serve as steering indicators. Upon identifying a red pillar, the system transmits signals to the Raspberry Pi, prompting a rightward steering adjustment. Conversely, detecting a green pillar triggers a leftward turn, ensuring precise directional control. The parking lot is consistently positioned within the starting section, allowing the robot to initiate its final maneuver upon completing three laps. At this stage, the system scans for magenta pixels, identifying the parking lot’s location. Following this step, the robot navigates toward the designated area, executing a controlled parking sequence. This integrated approach enhances autonomous navigation, obstacle recognition, and precise parking alignment, optimizing the robot’s overall performance.

Let’s dive into this mechanism deeper with the following steps:

**1. Starting The Robot:**
In the obstacle challenge, the robot's mechanism is activated by first turning on the main power switch, which initiates the circuit. The program is then manually started by pressing a push button. Following this boot-up sequence, the robot's image processing begins, using the same methodology as in the open challenge. This involves capturing and analyzing visual data to guide the robot's subsequent actions.

**2. Object and Wall Detection:**
Once the image is in the HSV format, the process_hsv() function analyzes the pixels to identify key elements:
•	Walls: Pixels with a low value (brightness less than 70) are identified as walls. The code calculates the density of these pixels on both the left and right sides of the frame, providing an estimate of the robot's proximity to a wall. This is a crucial input for the steering algorithm.
•	Traffic Lights: The script identifies red and green traffic lights based on their hue and saturation values. For instance, red traffic lights are identified by a hue value less than 15 or greater than 175, while green traffic lights are identified by a hue value between 45 and 90. These pixels are used to create separate masks for each color.
•	Parking Gate: A purple parking gate is detected using a similar method, identifying pixels within a specific hue, saturation, and value range to create a mask.

**3. Contour Analysis:** 
After the masks are generated, the script uses cv2.findContours() to identify contiguous shapes of each color. The process_traffic_contours() and process_parking_contours() functions then iterate through these shapes (contours) to find the largest one, which is assumed to be the target object. The x and y coordinates of the centroid of the largest contour are stored, along with its area and type (red or green).steps.

**4. Steering and Motor Control**
This is where the robot acts on the visual data. The steering direction is calculated using a proportional control (P-controller) algorithm. The Err variable, or error, is calculated based on the horizontal position of the detected traffic light. A positive error value indicates the traffic light is to the left of the center, and a negative value indicates it is to the right. The steering direction, dir, is then calculated as Err * kp, where kp is a constant gain of 0.30. This makes the steering proportional to the error, causing the robot to turn more sharply the farther it is from the center. The servo() function then takes this dir value and converts it into a PWM (Pulse Width Modulation) signal to control the servo motor, which adjusts the robot's wheels. The script also includes a fail-safe mechanism where if the wall density is too high, the robot will turn sharply away to prevent a collision.

**5. Mission Logic and Overrides:**
The code contains a complex state machine with various conditions that override the standard steering algorithm:
•	Quadrant Counting: The robot tracks its progress by counting blue and orange lines, which define different quadrants of the course.
•	Parking Sequence: In later quadrants, the script overrides the traffic light detection and initiates a new steering routine to align with and approach the parking gate. This routine uses the wall detection logic to align the robot parallel to the walls before stopping when the purple parking gate is close enough. The STOP variable is set to True to halt the robot and end the program once the mission is complete.

