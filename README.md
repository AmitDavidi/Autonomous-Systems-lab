## -Final- Path Control

### Description
This Project revolves around controling the movement of a robot using odometry and a Proportional-Integral-Derivative (PID) controller to achieve point-to-point (P2P) control. The robot is assumed to be equipped with encoders, motors, and an Inertial Measurement Unit (IMU) for measuring its orientation (Not used).

### Hardware Requirements
To run this code, you will need the following hardware components:
- Arduino board (e.g., Arduino Uno)
- Zumo32U4 robot chassis or similar robot platform
- Motors with encoders
- IMU sensor (Gyroscope) capable of measuring angular velocity - not used.

### Libraries Used
The code utilizes the following libraries:
- `Wire.h`: Used for I2C communication with the IMU.
- `Zumo32U4.h`: A custom library for the Zumo32U4 robot chassis, including motor and encoder control.

### Functionality
- The code enables the robot to follow a predefined path represented by a series of waypoints (pointsX and pointsY arrays). The robot calculates its position and orientation (odometry) using encoder readings and updates the motor commands to follow the desired path.
- Generation of the points here: https://colab.research.google.com/drive/1sQEZoUYBs30euKWlNJecnE6vXSFWy3KO?usp=sharing

### Variables and Parameters
The code includes various constants and variables to control the robot and perform calculations. Here are some of the key variables and their meanings:

- `SAMPLERATE`: The time interval (in milliseconds) for each control loop update.
- Odometry settings: Constants related to the robot's mechanical characteristics, such as gear ratio, wheel distance, wheel diameter, encoder pulses per revolution, and gyro scale.
- `pointsX` and `pointsY`: Arrays containing the x and y coordinates of the waypoints that define the desired path. You can modify these arrays to define your desired path.
- `NUM_POINTS`: The number of waypoints in the `pointsX` and `pointsY` arrays.
- Various control variables for motor speed and PID tuning parameters.

### Setup Function
- The `setup()` function is executed once when the Arduino is powered on or reset. It initializes the required components, sets the initial robot position and angle, and performs gyro calibration (commented out, since gyro isn't used).

### Loop Function
- The `loop()` function is the main control loop of the robot. It runs repeatedly, updating the robot's position and orientation, calculating control signals, and updating the motor speeds.

### P2P Control and PID Controller
- The function `P2P_CTRL()` calculates the desired motor speeds (LeftMotorSpeed_Cntrl_CMD and RightMotorSpeed_Cntrl_CMD) based on the desired position (X_Desired, Y_Desired) and the robot's current position and orientation. It follows a Proportional control approach to adjust the motor speeds.
- The function `pidController()` is a generic PID controller implementation used for both the left and right motor speed control.

### Gyro Integration (commented out)
- There are functions related to gyro integration and calibration (gyroIntegration() and gyroOffset()). These are commented out in the provided code.

### Odometry Function
- The function `odometry()` calculates the robot's position and orientation based on encoder readings and updates the `posx`, `posy`, and `theta` variables.

### Additional Notes
- The code assumes that the robot will move along the predefined path without any obstacles. It may require additional logic for obstacle avoidance and path planning for real-world applications.
- The control parameters (Kp, Ki, Kd) of the PID controller can be adjusted to optimize the robot's performance for different paths and robot configurations.
