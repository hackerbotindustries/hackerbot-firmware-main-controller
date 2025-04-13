# Main Controller Interface

This document outlines the serial commands used to interface with the **Main Controller** of the Hackerbot system. The controller manages communication with the robot’s core subsystems: **Base**, **Head**, and **Arm**. Each component accepts specific commands over a serial connection, allowing control over movement, navigation, and actuation.

---

## 📡 Communication Overview

Commands are sent as ASCII strings over a serial interface. Each command follows a structure:

```
<COMMAND> <PARAMETERS>
```

For example:
```
B_GOTO,2.1,-0.5,90.0,0.1
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
| `B_POSE`      | N/A                                  | Returns current pose (x, y, angle).               |
| `B_START`     | N/A                                  | Starts the base into dedicated mode.              |
| `B_QUICKMAP`  | N/A                                  | Performs quick mapping of environment.            |
| `B_DOCK`      | N/A                                  | Initiates docking procedure.                      |
| `B_KILL`      | N/A                                  | Emergency stop.                                   |
| `B_BUMP`      | left[`0` or `1`], right[`0` or `1`]  | Triggers bump-style movement.                     |
| `B_DRIVE`     | `linearVelocity, angularVelocity`    | Begins velocity controlled drive.                 |
| `B_GOTO`      | `x, y, angle, speed`                 | Navigates to a predefined map location.           |
| `B_MAPDATA`   | `map_id`                             | Retrieves current map data.                       |
| `B_MAPLIST`   | N/A                                  | Lists available maps.                             |

---

## 👀 Head Commands

| Command     | Parameters                                 | Description                                |
|-------------|---------------------------------------------|--------------------------------------------|
| `H_IDLE`    | `0` or `1`                                  | Puts head into idle or active state.       |
| `H_LOOK`    | `yaw, pitch, speed`                         | Directs the head to look in a direction.   |
| `H_GAZE`    | `x, y`                                      | Controls head gaze point.                  |

---

## 🤖 Arm Commands

| Command     | Parameters                                               | Description                                      |
|-------------|----------------------------------------------------------|--------------------------------------------------|
| `A_CAL`     | N/A                                                      | Calibrates the arm.                              |
| `A_OPEN`    | N/A                                                      | Opens the gripper.                               |
| `A_CLOSE`   | N/A                                                      | Closes the gripper.                              |
| `A_ANGLE`   | `joint, angle, speed`                                    | Moves a specific joint to an angle.              |
| `A_ANGLES`  | `j1, j2, j3, j4, j5, j6, speed`                          | Moves all joints simultaneously.                 |

---

## 📃 Example Commands

```bash
# Initializes the base driver
B_INIT

# Start the base to enter dedicated mode
B_START

# Move head to yaw=30.0, pitch=10.0 at speed 5
H_LOOK,30.0,10.0,5

# Gaze at a point x=-0.4, y=0.8
H_GAZE,-0.4,0.8

# Move arm joint 3 to 45 degrees at speed 10
A_ANGLE,3,45.0,10

# Move all arm joints to specific angles at speed 20
A_ANGLES,0.0,20.0,30.0,40.0,50.0,60.0,20
```

---

## 📂 License

MIT License

