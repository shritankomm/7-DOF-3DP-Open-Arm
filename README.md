# 7-DOF Open Source 3D Printed Humanoid Arms

An open-source, fully 3D-printable 7-DOF pair of humanoid robotic arms designed as an accessible research platform. Built with high-torque Feetech serial bus servos and a Raspberry Pi 4, it supports ROS2 for motion planning/simulation, VR teleoperation for intuitive control, and autonomous/agentic operation via an onboard camera. With all CAD files, BOM, and software fully open source, anyone can build and expand on it for under $800.

<img width="720" height="848" alt="image" src="https://github.com/user-attachments/assets/082effb3-57d0-4857-a68b-dcdca860cadb" />

---

## Inspiration
- I made this project because I find interest in is human-related robotics. The idea of being able to create some kind of device that can help or mimic a human's movement is a very cool thing, as human mechanics are very intricate. That's why I took the time to create as accurate a replica of the humans arms to make it work as well as possible. In the past, I have made foot prosthetics and other medical device prototypes, and this felt like a great next step. I also made this as an act to make a low-cost research tool for people to use, and as an ability for others and me to begin learning more advanced robotics. I'm exploring ROS, OpenCV, and other things that can be integrated with this project, and I feel this is the best way for people to learn things like this. I will document this journey and open-source it so others can follow and build upon the project, doing bigger, better, and more impressive things than I am with this project. I aim to inspire people to do more human-centered robotics, as it is the future. I have plans for more humanoid robot projects (a pair of legs, for example) and will document them on GitHub and make them open source so everyone can learn and grow as engineers.

---

## Features of the Project

- 7 Degrees of Freedom - full human-like range of motion across the shoulder, elbow, and wrist
- High-torque serial bus servos - 30 kgcm at most joints, 50kgcm at the shoulder
- Fully 3D printable - all structural components printed in PETG at high infill, no custom machined parts required
- ROS2 integration - built for compatibility with the broader robotics software ecosystem
- Advanced simulation using Gazebo 
- VR teleoperation - control the arm intuitively through a VR headset
- Autonomous and agentic control - onboard Raspberry Pi 4 and camera enable independent perception and decision-making
- Under $800 - accessible to students, researchers, and makers without big budgets


---

## Hardware

# Bill of Materials

| # | Part Name | Qty | Unit Price | Extended Price | Notes | Where It's Needed |
|---|-----------|-----|------------|-----------------|-------|--------------------|
| 1 | 1 KG PETG filament roll | 2 | $15.00 | $30.00 | | Needed to print out all the parts |
| 2 | STS3215 Servo (12V 30KG) | 14 | $24.00 | $336.00 | | Main actuators used at all joints except the top shoulder roll joint |
| 3 | Serial Bus Servo Driver Board | 2 | $11.00 | $22.00 | | Servo driver, supplies power to the arm and connects the Pi to send movement commands to the servos |
| 4 | Raspberry Pi 4 | 1 | $100.00 | $100.00 | | Microcontroller of the build, runs ROS and other programs |
| 5 | 12V 10A power supply | 1 | $23.00 | $23.00 | | Connects to the wall and supplies power to the build |
| 6 | M3 heat set inserts | 1 | $7.00 | $7.00 | Kit includes both screws and inserts; same price if bought separately | Inserts go into 3D prints to strengthen threads; screws fasten components together |
| 7 | Logitech C270 Webcam | 1 | $18.00 | $18.00 | For vision | USB cam for vision capabilities, lets the Pi make decisions based on vision |
| 8 | 4040 corner brackets + T-nuts | 1 | $20.00 | $20.00 | | Bottom piece of the extrusion that bolts to the table and holds the arm in place |
| 9 | 4040 extrusion, 800mm | 1 | $27.00 | $27.00 | | Extrusion bar needed to keep the arms elevated, acts as middle support |
| 10 | STS3250M | 2 | $75.00 | $150.00 | For the shoulder roll DOF (doesn't account for tariffs) | Servos for the shoulder pitch joints, higher torque |
| 11 | 5264 connector, 3-pin extended wires | 1 | $7.00 | $7.00 | | Wire extensions for the motors, allows proper daisy chaining in an open loop |
| 12 | 2x4, 60 inches | 1 | $5.00 | $5.00 | For the stand | For the stand |
| 13 | 3" wood screws | 8 | $6.00 | $6.00 | For the stand | For the stand |

**Total: $751.00**

Link to BOM Spreadsheet - https://docs.google.com/spreadsheets/d/1n2eNZaHsdRUR-5rRqqR8Au6a7Qyu3wT80XMrWUhEQkk/edit?usp=sharing

---

## Electrical

This is the preliminary wiring diagram for the project.

<img width="743" height="849" alt="image" src="https://github.com/user-attachments/assets/5452ae14-7f24-4c54-bd59-7601f64ba934" />


## Repository Structure

- CAD/          - CAD files (STL/STEP/Onshape Public Link), screenshots, Parts List, 3d print settings 
- software/     - Software + electrical diagram, firmware, and ROS2 control code
- docs/      - Build journal entries and progress updates/documentation
- BOM.md        - Full bill of materials with links and prices
- instructions.md - Build and assembly instructions 
- pictures.md - pictures of the fully assembled arm 
- README.md     - This file

---

## Build Instructions

Full step-by-step assembly instructions can be found in instructions.md. Follow along via the build journal in /DOCS to track design decisions and progress in the meantime. (coming soon)

---


## License
Hardware designs: CERN-OHL-S v2 (or CC BY 4.0)
