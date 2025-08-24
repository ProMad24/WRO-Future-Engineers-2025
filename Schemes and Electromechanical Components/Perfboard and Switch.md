# Perfboard // Switch:

- The Perfboard in our robot serves as the central hub for electrical connectivity, linking all major components into a cohesive and efficient system. It features two input pins designated for the positive and negative poles of the LiPo battery series, which supply the primary 12V power source. Additionally, the board includes a dedicated row of GND pins that are directly connected to the Raspberry Pi 4 Model B, ensuring a stable ground reference across all subsystems. To distribute power effectively, the perfboard provides six output pins for 12V and six output pins for 5V, the latter being regulated through the Mini 360 DC-DC Buck Converter.

Furthermore, the perfboard integrates a USB Type-A converter module, which plays a crucial role in delivering a regulated 5V supply to the Raspberry Pi 4 Model B. This setup not only simplifies the wiring architecture but also enhances the reliability and safety of the robot’s power distribution. By consolidating power inputs, outputs, and grounding into a single board, the perfboard ensures that all components—from servo motors to LEDs—operate smoothly and in coordination, making it an essential element of our robotic system.

- And surely, we use a switch to close or open the circuit.
