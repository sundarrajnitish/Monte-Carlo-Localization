function result = FunctionParticleFilter(robotDataFileLocation, mapDataFileLocation, likelihoodMapFileLocation)

robotData = dlmread(robotDataFileLocation);
mapData = dlmread(mapDataFileLocation);
likelihoodMapData = dlmread(likelihoodMapFileLocation);

% No. of Particles
numParticles = 500;

% Storing X, Y, theta and Weight of particles
particles = zeros(numParticles, 4);

x_range = 72:85;
y_range = 445:465;
theta_range_1 = -pi:(pi + 0.1745); 
theta_range_2 = (pi - 0.1745):pi;

for i = 1:numParticles
    particles(i, 1) = datasample(x_range, 1);
    particles(i, 2) = datasample(y_range, 1);
    particles(i, 3) = datasample([theta_range_1, theta_range_2], 1);
    particles(i, 4) = 1;
end

robotPose = [];

currentRobotX = 445;
currentRobotY = 72;
currentRobotTheta = 0;

% Initialize a scatter plot with the first set of data points
figure;
scatter(particles(:, 1), particles(:, 2)); % Plot x vs y
title('Scatter Plot');
xlabel('X');
ylabel('Y');

for i = 1:size(robotData, 1)
    if robotData(i, 1) == 0
        %Updated Robot Position using the odometry model
        [currentRobotX, currentRobotY, currentRobotTheta] = FunctionOdometryModel(robotData(i, :), currentRobotX, currentRobotY, currentRobotTheta);
        robotPose = [robotPose; [currentRobotX, currentRobotY, currentRobotTheta]];

        % Propagating the particles using the odometry model
        for j = 1:numParticles
            particles(j, 1) = particles(j, 1) + currentRobotX;
            particles(j, 2) = particles(j, 2) + currentRobotY;
            particles(j, 3) = particles(j, 3) + currentRobotTheta;
        end

    elseif robotData(i, 1) == 1

        % Updated Robot Position using the odometry model

        [currentRobotX, currentRobotY, currentRobotTheta] = FunctionOdometryModel(robotData(i, :), currentRobotX, currentRobotY, currentRobotTheta);
        robotPose = [robotPose; [currentRobotX, currentRobotY, currentRobotTheta]];

        for j = 1:numParticles
            %particles(j, 4) = FunctionSensorModelBeamBasedModel(robotData(i, 5:7), mapData, robotData(i, 8:187));
            particles(j, 4) = FunctionSensorModelBeamBasedModel(robotData(i, 5:7), likelihoodMapData, robotData(i, 8:187));
        end

        % Propagating the particles using the odometry model
        for j = 1:numParticles
            particles(j, 1) = particles(j, 1) + currentRobotX;
            particles(j, 2) = particles(j, 2) + currentRobotY;
            particles(j, 3) = particles(j, 3) + currentRobotTheta;
        end

    end
    
    x = particles(:, 1);
    y = particles(:, 2);
    
    % Update the scatter plot with new x, y coordinates
    scatter(x, y, 'r', 'filled'); % Plot the current point in red
    
    % Update title to show the iteration number
    title(['Scatter Plot - Iteration ', num2str(i)]);

    pause(0.1);

    particles = FunctionLowVarianceResample(particles);
    
end

result = particles;

end

