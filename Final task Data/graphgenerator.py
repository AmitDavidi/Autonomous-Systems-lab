import matplotlib.pyplot as plt

# Read the text file
with open('Square Data for Final Repot.txt', 'r') as file:
    lines = file.readlines()

# Extract x and y coordinates from each line
x_coords = []
y_coords = []

for line in lines:
    # Extract pos values from each line
    start_index = line.find("pos = (") + len("pos = (")
    end_index = line.find(")", start_index)
    pos_str = line[start_index:end_index]

    # Split x and y coordinates
    x, y = map(float, pos_str.split(','))
    x_coords.append(x)
    y_coords.append(y)

# Plot the locations
plt.scatter(x_coords, y_coords)
plt.xlabel('X[m] Position', fontsize = 20)
plt.ylabel('Y[m] Position', fontsize = 20)
plt.title('2D Position Plot', fontsize = 20)
plt.grid(True)
plt.show()


left_motor_ctrl = []
right_motor_ctrl = []

for line in lines:
    # Extract pos values from each line
    start_index = line.find("P2P = (") + len("P2P = (")
    end_index = line.find(")", start_index)
    pos_str = line[start_index:end_index]

    # Split left and right
    x, y = map(float, pos_str.split(','))
    left_motor_ctrl.append(x)
    right_motor_ctrl.append(y)

# Plot the locations
plt.plot(left_motor_ctrl, label='Left Motor Command from P2P - Desired Velocity')
plt.plot(right_motor_ctrl, label='Right Motor Command from P2P - Desired Velocity')



left_motor_vel = []
right_motor_vel = []

for line in lines:
    # Extract pos values from each line
    start_index = line.find("Speed = (") + len("Speed = (")
    end_index = line.find(")", start_index)
    pos_str = line[start_index:end_index]

    # Split left and right
    x, y = map(float, pos_str.split(','))
    left_motor_vel.append(x)
    right_motor_vel.append(y)

# Plot the locations
plt.plot(left_motor_vel, label='Left Motor Velocity')
plt.plot(right_motor_vel, label='Right Motor Velocity')

plt.xlabel('# Measurement', fontsize = 20)
plt.ylabel('velocity [m/s]', fontsize = 20)
plt.title('Motor Control commands and actual Velocity', fontsize = 20)
plt.grid(True)
plt.legend(fontsize = 20)
plt.show()
