function [currentX, currentY, currentTheta] = FunctionOdometryModel(data, prevX, prevY, prevTheta)

    currentX = data(1, 2);
    currentY = data(1, 3);
    currentTheta = data(1, 4);

    % alpha
    alpha = 1*10^-6;

    % Delta Translation
    delTrans = sqrt((currentX - prevX)^2 + (currentY - prevY)^2);

    %Odometry Corrections
    thetaPiCrossover = currentTheta - prevTheta;

    if (thetaPiCrossover < -pi)

        thetaPiCrossover = thetaPiCrossover + 2*pi;

    elseif (thetaPiCrossover > pi)

        thetaPiCrossover = thetaPiCrossover - 2*pi;
    end


    if delTrans == 0
        % Delta Rotation 1
        delRot1 = 0;

        % Delta Rotation 2
        delRot2 = prevTheta - currentTheta;
    else
        % Delta Rotation 1
        if currentX > 0
            
        end
        delRot1 = atan2(currentY - prevY, currentX - prevX) - prevTheta;

        % Delta Rotation 2
        delRot2 = thetaPiCrossover - delRot1;
    end

    % Noise Model for Odometry
    NdelRot1 = normrnd(delRot1, alpha*(delRot1^2) + alpha*(delTrans^2));

    NdelTrans = normrnd(delTrans, alpha*(delTrans^2) + alpha*((delRot1^2) + (delRot2^2)));

    NdelRot2 = normrnd(delRot2, alpha*(delRot2^2) + alpha*(delTrans^2));

    % Current Robot Co-ordinates
    currentX = prevX + NdelTrans*(cos(prevTheta + NdelRot1));

    currentY = prevY + NdelTrans*(sin(prevTheta + NdelRot1));

    currentTheta = prevTheta + NdelRot1 + NdelRot2;

end

