import matplotlib.pyplot as plt

# Read the text file
with open('square115fix.txt', 'r') as file:
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
plt.xlabel('X[m] Position')
plt.ylabel('Y[m] Position')
plt.title('2D Position Plot')
plt.grid(True)
plt.show()
