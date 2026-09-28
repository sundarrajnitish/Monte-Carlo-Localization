function weight = FunctionSensorModelLikelihoodFieldModel(robotData, mapData, lazerScans)
    
    occupied_grid = mapData;

    weights = [];

    robotLazerTheta = robotData(1, 3);

    start_angle = robotLazerTheta - 1.56206968;
    end_angle = robotLazerTheta + 1.56206968;

    % Generate a sequence of values with a step size of 1° within the specified range
    angle_range = linspace(start_angle, end_angle, 180);

    x = robotLazerX;
    y = robotLazerY;

    xOccupied = -1;
    yOccupied = -1;

    for beam = 1:size(lazerScans, 2)

        robotLazerX = robotData(1, 1) + 25*(cos(angle_range(beam)));
        robotLazerY = robotData(1, 2) + 25*(sin(angle_range(beam)));

        xCell = floor(robotLazerX/4);
        yCell = floor(robotLazerY/4);

        if xCell > 0 && xCell <= size(occupied_grid, 2) && yCell > 0 && yCell <= size(occupied_grid, 1)
             pHit  = occupied_grid(xCell, yCell);
        else
            pHit = -1;
        end
       
        pShort = (1/(1-(exp(-0.1*8000))))*(0.1*exp(-0.1*(lazerScans(1, beam))));

        if (lazerScans(1, beam)<8000)
            pMax = 0;
        else
            pMax = 8000;
        end

        pRand = 1/8000;

        pProduct =(0.9*pHit) + (0.09*pShort) + (0.01*pMax) + (0.001*pRand);
        weights = [weights; pProduct];
    end

    weight = prod(weights, "all");
end



% Apply a logarithmic transformation to enhance the gradual differences
%log_transformed_data = log(1 + dataset * 100);

% Normalize the transformed dataset values to the range [0, 1] for grayscale mapping
%normalized_data = (log_transformed_data - min(log_transformed_data(:))) / (max(log_transformed_data(:)) - min(log_transformed_data(:)));

% Plotting the enhanced grayscale plot as an image
%figure;
%imagesc(normalized_data); % Plot the enhanced grayscale image
%colormap(gray); % Set the colormap to grayscale

% Customize the plot
%colorbar; % Show colorbar to indicate the values
%xlabel('Columns');
%ylabel('Rows');
%title('Likelihood Field Map');