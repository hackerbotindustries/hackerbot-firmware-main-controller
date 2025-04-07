# Main Controller Interface

This document outlines the serial commands used to interface with the **Main Controller** of the Hackerbot system. The controller manages communication with the robot’s core subsystems: **Base**, **Head**, and **Arm**. Each component accepts specific commands over a serial connection, allowing control over movement, navigation, and actuation.

---

## 📡 Communication Overview

Commands are sent as ASCII strings over a serial interface. Each command follows a structure:

```
<Component>_<COMMAND> <PARAMETERS>
```

For example:
```
B_GOTO 1.0,2.0,90.0,1
```

---

## 🧱 Base Commands

| Command       | Parameters                          | Description                                       |
|---------------|--------------------------------------|---------------------------------------------------|
| `PING`        | N/A                                  | Check the base is responsive and its connections. |
| `VERSION`     | N/A                                  | Returns firmware version of the base module.      |
| `JSON`        | `0` or `1`                           | Toggles JSON formatted responses.                 |
| `TOFS`        | `0` or `1`                           | Toggles ToF sensors.                              |
| `B_INIT`      | N/A                                  | Initializes the base driver.                      |
| `B_MODE`      | `0 - 12`                             | Sets operational mode.                            |
| `B_STATUS`    | N/A                                  | Retrieves current status.                         |
| `B_POSE`      | N/A                                  | Returns current pose (x, y, theta).               |
| `B_START`     | `X,Y,Angle,Rotation`                 | Starts the base into dedicated mode.              |
| `B_QUICKMAP`  | N/A                                  | Performs quick mapping of environment.            |
| `B_DOCK`      | N/A                                  | Initiates docking procedure.                      |
| `B_KILL`      | N/A                                  | Emergency stop.                                   |
| `B_BUMP`      | `LinearVelocity,AngularVelocity`     | Triggers bump-style movement.                     |
| `B_DRIVE`     | N/A                                  | Begins velocity controlled drive.                 |
| `B_GOTO`      | `MapID`                              | Navigates to a predefined map location.           |
| `B_MAPDATA`   | N/A                                  | Retrieves current map data.                       |
| `B_MAPLIST`   | N/A                                  | Lists available maps.                             |

---

## 👀 Head Commands

| Command     | Parameters                                 | Description                                |
|-------------|---------------------------------------------|--------------------------------------------|
| `H_IDLE`    | `0` or `1`                                  | Puts head into idle or active state.       |
| `H_LOOK`    | `Yaw,Pitch,Speed`                           | Directs the head to look in a direction.   |
| `H_GAZE`    | `X,Y`                                       | Controls head gaze point.                  |
| `H_POSE`    | N/A                                         | Returns the current head pose.             |

---

## 🤖 Arm Commands

| Command     | Parameters                                               | Description                                      |
|-------------|----------------------------------------------------------|--------------------------------------------------|
| `A_CAL`     | N/A                                                      | Calibrates the arm.                              |
| `A_OPEN`    | N/A                                                      | Opens the gripper.                               |
| `A_CLOSE`   | N/A                                                      | Closes the gripper.                              |
| `A_ANGLE`   | `Joint,Angle,Speed`                                      | Moves a specific joint to an angle.              |
| `A_ANGLES`  | `J1,J2,J3,J4,J5,J6,Speed`                                | Moves all joints simultaneously.                 |
| `A_POSE`    | N/A                                                      | Returns the current pose of the arm.             |

---

## 📃 Example Commands

```bash
# Start the base at position x=1.0, y=2.0, angle=90 degrees, rotation=1
B_START 1.0,2.0,90.0,1

# Move head to yaw=30.0, pitch=10.0 at speed 5
H_LOOK 30.0,10.0,5

# Gaze at a point x=1.5, y=0.8
H_GAZE 1.5,0.8

# Move arm joint 3 to 45 degrees at speed 10
A_ANGLE 3,45.0,10

# Move all arm joints to specific angles at speed 20
A_ANGLES 10.0,20.0,30.0,40.0,50.0,60.0,20
```

---

## 📂 License

MIT License

