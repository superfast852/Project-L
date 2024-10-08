# Project L
### A Competitive, Capable robot based on Mecanum and Lidar.

---
#### Table of Contents
1. [Introduction](#introduction)
2. [Hardware](#hardware)
3. [Software](#software)
4. [TODO](#todo)
5. [Goals](#goals)

---
## Introduction
Project L is a Custom General Purpose Robot designed with automation, expandability and speed in mind. It is based on a 4-wheel drive chassis with mecanum wheels. It is designed to be able to navigate and perform tasks autonomously, and to be interactive and easily expanded upon in software and hardware.
---
## Hardware
L is equipped with a 4-Wheel Mecanum Drive, as well as a LiDAR Sensor for navigation and a 5 DOF Robot Arm for manipulation.

---
## Software
L runs on an integrated computer (RasPI or Orin Nano), and features a high level control interface for the hardware as seen in the Robot class and the HAL (Hardware Abstraction Layer).
It communicates using NetworkTables, which automatically gives it support for FRC Dashboards (Shuffleboard is preffered) and Libraries.
The main files to run are main.py, which goes in the robot itself, and remote.py, which is the remote control and management app.

---
## TODO:
####    Drive:
- [ ] Linking PCB Build
- [ ] Get the damn parts

####    Power:
- [ ] Power Regulation and Distribution block: A block that takes in 12V and 
outputs every power connection. Includes Battery voltage monitoring and cutoff. Uses an arduino to transmit data.

####    Interfacing:
- [X] Encoder Handling
- [-] Video Feed (we got FastCam handler)
- [X] Arm Control
- [-] Lidar Handling
- [X] Drive Communication
- [X] Drive Handling
- [X] Voltage Monitoring
- [X] Ultrasonic Interface
- [X] MPU-9250 Integration

####    Software:
- [-] Networking (Network Tables!)
- [ ] Sensor Interface
- [-] Proper Mapping
- [-] Autonomous
- [ ] Low Level -> High Level Interface 
- [ ] Given Path Following
- [X] A*/RRT Path Planning
- [X] SLAM (See [BreezySLAM](https://github.com/simondlevy/breezyslam))
- [X] Drive Integration
- [-] HMI-Like Control Interface using Shuffleboard
- [ ] Pure Pursuit
- [ ] Test out the code
- [ ] Frontier Exploration (and adopt Transient Maps in general :/)
####    Expansion:
- [ ] Find out how to enable expansion (the hal is pretty nifty in that sense imo)

####    Comms:
- [X] Local Communication (Drive -> Pi <- PDB) (It's now all in one (rosmaster board :))
- [X] Remote Communication (Pi -> Driver Station) (pynetworktables)
- [X] Long-Term Data Storage (Maps, Logs, etc.) (TODO: create a robot state file to save the bot's state at a certain point)

####    Base:
- [X] Get the damn base
- [ ] Expandability Standard

---
## Goals
- [-] Automatic Navigation
- [ ] Speed
- [-] A-B Navigation on prerecorded map
- [-] SLAM
- [-] Path Planning
- [-] Task Creation Framework (Have to properly implement PolyTask)

---
## Links
- [BreezySLAM](https://github.com/simondlevy/breezyslam)
- [Parts List](https://docs.google.com/spreadsheets/d/1OO8v4pfx6eXCZQJqTUq6JUSEajtX1vfl/edit?usp=sharing&ouid=107364809967877055034&rtpof=true&sd=true)

---
## Notes:
+ We should be figuring out how to integrate dot-by-dot path following. This allows for bezier curves and smooth paths.


### Current development:
- PID (Speed and Positioning)
- Path Following (Pure Pursuit or Custom Algorithm)
- PolyTask (da bestest :)
- Communications (pynetworktables or local server, we'll see)
- GUI
- Odometry and Positioning (Unify encoder position with lidar position plz thx :)
- CHECK THE TODOs
- Fix Odometry so that we have a travel and global position.
- Standardize Maps to work with meters, so that simulation and real-to-digital conversion can be easier and more comprehensible.
