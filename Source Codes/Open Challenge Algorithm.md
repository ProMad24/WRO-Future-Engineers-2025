# 1. Open Challenge: 

To address the challenge, we developed a camera-based algorithm utilizing the Raspberry Pi 4 Model B alongside the Pi Camera Model 3. This system enables precise linear alignment, ensuring the robot maintains a consistent trajectory along the mat’s walls throughout the four straightforward sections. The steering mechanism is dynamically adjusted based on color recognition, allowing the robot to detect and respond to walls when encountering a corner. Additionally, the system incorporates a lap-counting function relying on the strips in corners, enabling the robot to track its completed laps before executing a stop command. This integrated approach enhances autonomous navigation, spatial awareness, and real-time decision-making, optimizing the robot’s overall performance.

To further elaborate on this mechanism, the following is a step-by-step breakdown of its functionality:

### *__1.1.__ Starting The Robot:*
- Once the robot is placed on the mat, we close the robot’s circuit using a simple switch to provide it with power. Then, it initiates the program execution with a button press, activating its driving mechanism.
------
### *__1.2.__	Image Processing:*
- The Pi Camera captures an image of the robot’s field of view, transmitting it to the Raspberry Pi for processing.

- Much like the human eye, the camera is positioned to capture a flipped image at a resolution of 640×480 pixels. Upon receiving the photograph from the PiCamera 3, the Rpi4 immediately initiates the processing sequence. This sequence involves flipping the image, adjusting its resolution, and performing color detection.

- With more details, the Rpi reduces the resolution to 320×120 pixels and removes the top 40 rows to isolate the area containing the mat. Keeping in mind that it structures the pixel data into arrays using Python, while leveraging the NumPy library for efficient manipulation. This technique not only facilitates resolution reduction but also contributes to image inversion, as the pixels are reordered in reverse to produce the correct visual phase.

- We arrive at the final stage of the image processing sequence: color detection. The Rpi is responsible for identifying a variety of colors by relying on the HSV (Hue, Saturation, Value) color model, which ensures optimal detection accuracy. Specifically, the algorithm recognizes walls as black pixels, while blue and orange pixels indicate corner strips.
  
  - Moreover, color detection is contingent upon a threshold condition being met. If the pixel count does not surpass this threshold, the robot disregards the orange and blue corner strips. However, if a strip is successfully detected, its state is set to 1, indicating active detection. Once the strip moves out of view, the state transitions to 2, initiating a cooldown phase before the next strip can be registered.
  - Additionally, the algorithm designed for linear alignment relies on detecting the proportional presence of both the left and right walls. To facilitate this, the Rpi splits the image into two halves—one representing the left wall’s black pixels, and the other capturing the right wall’s black pixels—allowing the system to evaluate wall alignment more accurately based on distribution and positioning.
---
### *__1.3.__	Analysis of Given Information and Solving the Challenge:*
- As the Rpi4 interprets the robot’s field of view, it issues optimal commands to execute the designated task. The following outlines these commands step by step:

- <ins>*__Step One:__*</ins> Upon activation—triggered by pressing the start button—the DC motor immediately begins rotating at a speed of 90 RPM. Simultaneously, the PiCamera initiates image analysis.

- <ins>*__Step Two:__*</ins> The PiCamera processes 30 frames per second. For each frame, the Rpi4 evaluates the distribution of black pixels across both halves of the image. It then performs proportional calculations to assess the robot’s alignment relative to the surrounding walls and its distance from the outer boundary. If the robot is properly aligned, the Rpi4 maintains the steering wheels in a forward position without altering the angle. However, if any misalignment is detected, it adjusts the steering angle via the Servo motor using a proportional (P) controller to restore proper orientation.

- <ins>*__Step Three:__*</ins> When the robot encounters a corner, an additional condition is applied to the previous process. Specifically, if the Rpi4 detects an insufficient number of black pixels on either half of the frame, it infers the absence of a wall and authorizes an increase in the steering angle. The direction of the Servo motor’s movement—whether it turns left or right—is determined by the color of the first strip identified: orange strips signal a clockwise turn (positive angle change), while blue strips indicate a counter-clockwise turn (negative angle change).
  - Furthermore, these colored strips serve as corner counters. Each time the robot detects a strip, it increments its corner count by one. Once the count reaches twelve, the robot terminates its operation. Plus, to prevent the simultaneous detection of both strips, a brief delay is introduced following the observation of one.

- <ins>*__Added Feature:__*</ins> An Assembly LED was installed to provide visual feedback on the robot’s readings. It reflects key states such as the detection of different colors. Further details are addressed in a separate directory.
