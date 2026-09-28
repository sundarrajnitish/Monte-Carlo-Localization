map_data = dlmread("../Data/map2023DAT.dat");

% Assuming 'map_data' contains your map and the robot's initial position and angle are known
robot_initial_x_range = 445:465;
robot_initial_y_range = 72:85;
robot_initial_theta_range = [-0.1745, 0.1745]; % Range of initial theta in radians

% Global map orientation (adjust this as per your map)
global_map_orientation = 36; % Set the global map's orientation in radians

% Define the size of a grid square in centimeters
grid_size_cm = 4;

% Calculate the size of the map in centimeters
map_width_cm = size(map_data, 2) * grid_size_cm;
map_height_cm = size(map_data, 1) * grid_size_cm;

% Create a grid of x and y coordinates for plotting
x = linspace(0, map_width_cm, size(map_data, 2) + 1);
y = linspace(0, map_height_cm, size(map_data, 1) + 1);

% Display the map
imagesc(x, y, map_data);
colormap([1 1 1; 0 0 0]); % Define colormap for 0s (white) and 1s (black)
xlabel('Width (cm)');
ylabel('Height (cm)');
title('Map Visualization');
axis image; % Set aspect ratio to be equal
colorbar; % Show color bar indicating values (0 and 1)

% Overlay the robot's initial position and orientation on the map
hold on;

% Calculate the center of the robot's initial position range
robot_initial_x = mean(robot_initial_x_range) * 4; % Convert grid index to cm
robot_initial_y = mean(robot_initial_y_range) * 4; % Convert grid index to cm

% Calculate the absolute angles of the robot relative to the global map for both bounds
absolute_robot_theta_lower = global_map_orientation + robot_initial_theta_range(1, 1);
absolute_robot_theta_upper = global_map_orientation + robot_initial_theta_range(1, 2)*2;

% Plot a marker indicating the robot's initial position
plot(robot_initial_x, robot_initial_y, 'sq', 'MarkerSize', 10, 'Color','red'); % Red circle marker

% Calculate the end coordinates of the lines representing the robot's orientation for both bounds
arrow_length = 150; % Length of the arrows representing the orientation

% Lower bound orientation arrow
arrow_end_x_lower = robot_initial_x + arrow_length * cos(absolute_robot_theta_lower);
arrow_end_y_lower = robot_initial_y + arrow_length * sin(absolute_robot_theta_lower);
quiver(robot_initial_x, robot_initial_y, arrow_end_x_lower - robot_initial_x, arrow_end_y_lower - robot_initial_y, 0, 'LineWidth', 1, 'MaxHeadSize', 0.5, 'Color', 'blue');

% Upper bound orientation arrow
arrow_end_x_upper = robot_initial_x + arrow_length * cos(absolute_robot_theta_upper);
arrow_end_y_upper = robot_initial_y + arrow_length * sin(absolute_robot_theta_upper);
quiver(robot_initial_x, robot_initial_y, arrow_end_x_upper - robot_initial_x, arrow_end_y_upper - robot_initial_y, 0, 'LineWidth', 1, 'MaxHeadSize', 0.5, 'Color', 'blue');

hold off;