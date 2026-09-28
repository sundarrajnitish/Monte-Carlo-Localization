function w = sensor_model(x)

    sigma_door = 0.2;
    w = 0.05+0.9*(exp(-(x-3).^2./2/(sigma_door)^2)+exp(-(x-5).^2./2/(sigma_door)^2)+exp(-(x-10).^2./2/(sigma_door)^2));