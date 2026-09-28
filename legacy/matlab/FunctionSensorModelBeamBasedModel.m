function [weight] = FunctionSensorModelBeamBasedModel(robotData, mapData, lazerScans)

    occupied_grid = mapData;

    robotLazerX = robotData(1, 1) + 25*(cos(robotData(1, 3)));
    robotLazerY = robotData(1, 2) + 25*(sin(robotData(1, 3)));
    robotLazerTheta = robotData(1, 3);

    weights = [];

    start_angle = robotLazerTheta - 1.56206968;
    end_angle = robotLazerTheta + 1.56206968;

    % Generate a sequence of values with a step size of 1° within the specified range
    angle_range = linspace(start_angle, end_angle, 180);

    x = robotLazerX;
    y = robotLazerY;

    xOccupied = -1;
    yOccupied = -1;

    for beam = 1:size(lazerScans, 2)
        slope = tan(angle_range(beam));
        lazerHit = false;
        while ~lazerHit
            
            xCell = floor(x/4);
            yCell = floor(y/4);

            if xCell > 0 && xCell <= size(occupied_grid, 2) && yCell > 0 && yCell <= size(occupied_grid, 1)
                if occupied_grid(yCell, xCell) == 1

                    xOccupied = x;
                    yOccupied = y;
                    lazerHit = true;
                    break;
                end
            else
                % Out of bounds
                lazerHit = true;
            end

            if slope<1
                x = x + 4;
                y = y + 4*(slope);
            elseif slope>1
                y = y + 4;
                x = x + (4/slope);
            end
        end

        zExp = sqrt(((xOccupied - robotLazerX)^2) + ((yOccupied - robotLazerY)^2));

        pHit = (1/(normcdf(8000,zExp,5)-normcdf(0,zExp,5)))*(1/(sqrt(2*pi*(5^2))))*exp((-1/2)*(((lazerScans(1, beam)-zExp)^2)/(5^2)));
        
        if (pHit < 0.0001)
            pHit = 0;
        end

        if (lazerScans(1, beam)>zExp)
            pShort = 0;
        else
            pShort = (1/(1-(exp(-0.1*zExp))))*(0.1*exp(-0.1*(lazerScans(1, beam))));
        end

        if (lazerScans(1, beam)~=zExp || lazerScans(1, beam)<8000)
            pMax = 0;
        else
            pMax = 8000;
        end

        pRand = 1/8000;

        pProduct =(0.9*pHit) + (0.09*pShort) + (0.01*pMax) + (0.001*pRand);
        weights = [weights; pProduct];
    end

    weight = sum(weights, "all");
end