# 7-DOF Open Source 3D Printed Humanoid Arms

An open-source, fully 3D-printable 7-DOF humanoid robotic arm designed as an accessible research platform. Built with high-torque Feetech serial bus servos and a Raspberry Pi 4, it supports ROS2 for motion planning/simulation, VR teleoperation for intuitive control, and autonomous/agentic operation via an onboard camera. With all CAD files, BOM, and software fully open source, anyone can build and expand on it for under $800.

<img width="621" height="754" alt="image" src="https://github.com/user-attachments/assets/43c9c1ac-a309-42b5-b6e7-95668deaa1e7" />

---

## Inspiration
- I made this project as something I find interest in is human-related robotics. The idea of being able to create some kind of device that can help or mimic a human's movement is a very cool thing, as human mechanics are very intricate. That's why I took the time to create as accurate a replica of the humans arms to make it work as well as possible. In the past, I have made foot prosthetics and other medical device prototypes, and this felt like a great next step. I also made this as an act to make a low-cost research tool for people to use, and as an ability for others and me to begin learning more advanced robotics. I'm exploring ROS, OpenCV, and other things that can be integrated with this project, and I feel this is the best way for people to learn things like this. I will document this journey and open-source it so others can follow and build upon the project, doing bigger, better, and more impressive things than I am with this project. I aim to inspire people to do more human-centered robotics, as it is the future. I have plans for more humanoid robot projects (a pair of legs, for example) and will document them on GitHub and make them open source so everyone can learn and grow as engineers.

---

## Features of the Project when Finished

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

All parts, links, and pricing can be found in BOM folder, as a csv file and as a google sheets link.

Link to BOM Spreadsheet - https://docs.google.com/spreadsheets/d/1n2eNZaHsdRUR-5rRqqR8Au6a7Qyu3wT80XMrWUhEQkk/edit?usp=sharing

---

## Electrical

This is the preliminary wiring diagram for the project.

<img width="743" height="849" alt="image" src="https://github.com/user-attachments/assets/5452ae14-7f24-4c54-bd59-7601f64ba934" />


## Repository Structure

- CAD/          - CAD files (STL/STEP/Onshape Public Link), screenshots, Parts List, 3d print settings 
- software/     - Software + electrical diagram, firmware, and control code (coming soon)
- docs/      - Build journal entries and progress updates/documentation
- BOM.md        - Full bill of materials with links and prices
- instructions.md - Build and assembly instructions 
- pictures.md - pictures of the fully assembled arm 
- README.md     - This file

---

## Build Instructions

Full step-by-step assembly instructions can be found in instructions.md. Follow along via the build journal in /DOCS to track design decisions and progress in the meantime. (coming soon)

---

## Coming Soon

- ROS2 packages and launch files
- Vision capabilities 
- VR teleoperation integration
- Full open-source firmware and control code
- Complete build instruction guide

## License
Hardware designs: CERN-OHL-S v2 (or CC BY 4.0)
