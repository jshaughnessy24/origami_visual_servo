folder_path = "~/origami_visual_servo/ros2_ws/src/projects/origami_arm/exp_log/";
% Read module status data
file_name = folder_path + "end_flag.csv";
status_data = readtable(file_name);

% Remove time stamps & data offset
status_data = status_data(:,4:end);

% Find status change event
module2 = 1;
for row=1:height(status_data)
    if strcmp(status_data{row,:}{1,1},'True') && module2==1
        module2 = row;
    end
end

% Read image error data
file_name = folder_path + "marker_coordinates_image_errors.csv";
error_data = readtable(file_name);

% Remove time stamps & data offset
error_data = error_data(:,4:end);
% Compute error norm
error_norm = [];
for row = 1:height(error_data)
    newsubstr = char(extractBetween((error_data{row,:}{1,1}), "[", "]"));
    newsub2 = strrep(newsubstr,',','');
    coord_error_arr = sscanf(char(newsub2), '%f');
    error_norm = [error_norm, norm(coord_error_arr)];
end

figure(1)
hold on
plot(error_norm,'LineWidth',3.0)

title('Image Errors Norm')
ylabel('Image Error (pixels)')
xlabel('Iteration at 10 Hz')

% Save plot to file
save_path = folder_path + "error_norm";
saveas(gcf,save_path, 'png')


% Read velocity data
file_name = folder_path + "xz_cam_desired_velocity.csv";
error_data = readtable(file_name);

% Remove time stamps & data offset
error_data = error_data(:,4:end);

% Compute velocity norm
error_norm = [];
for row = 1:height(error_data)
    newsubstr = char(extractBetween((error_data{row,:}{1,1}), "[", "]"));
    newsub2 = strrep(newsubstr,',','');
    coord_error_arr = sscanf(char(newsub2), '%f');
    error_norm = [error_norm, norm(coord_error_arr)];
end

figure(2)
hold on
plot(error_norm,'LineWidth',3.0)
title('x,y,z Instantaneous Camera Velocities Norm')
ylabel('Instantaneous Camera Velocities Norm (mm/sec) LT^{-1}')
xlabel('Iteration at 10 Hz')

% Save plot to file
save_path = folder_path + "xyz_error_norm";
saveas(gcf,save_path, 'png')

% Read motor velocity data
file_name = folder_path + "motor_desired_velocities.csv";
velocity_data = readtable(file_name);

% Remove time stamps & data offset
velocity_data = velocity_data(:,4:end);

velocity_array = [];
% Get array
for row = 1:height(error_data)
    newsubstr = char(extractBetween((velocity_data{row,:}{1,1}), "[", "]"));
    newsub2 = strrep(newsubstr,',','');
    velocity_array = [velocity_array; sscanf(char(newsub2), '%f', [1 3])];
end

velocity_data_module1 = velocity_array;

figure(3)
plot(velocity_data_module1(:,1),LineWidth=3.0)
hold on
plot(velocity_data_module1(:,2),LineWidth=3.0)
plot(velocity_data_module1(:,3),LineWidth=3.0)
title('Actuator Velocities Module 1')
ylabel('Tendon Velocity (rpm) aT^{-1}')
xlabel('Iteration at 10 Hz')
legend('1', '2', '3')

% Save plot to file
save_path = folder_path + "velocity";
saveas(gcf,save_path, 'png')

% Read velocity data
file_name = folder_path + "xz_cam_desired_velocity.csv";
image_velocity_data = readtable(file_name);

% Remove time stamps & data offset
image_velocity_data = image_velocity_data(:,4:end);

image_velocity_array = [];

% Get array
for row = 1:height(error_data)
    newsubstr = char(extractBetween((image_velocity_data{row,:}{1,1}), "[", "]"));
    newsub2 = strrep(newsubstr,',','');
    image_velocity_array = [image_velocity_array; sscanf(char(newsub2), '%f', [1 4])];
end

image_velocity_data_module1 = image_velocity_array;

figure(4)
plot(image_velocity_data_module1(:,1),LineWidth=3.0)
hold on
plot(image_velocity_data_module1(:,2),LineWidth=3.0)
plot(image_velocity_data_module1(:,3),LineWidth=3.0)
title('x,y,z Instantaneous Camera Velocities')
ylabel('Instantaneous Camera Velocities (mm/sec) LT^{-1}')
xlabel('Iteration at 10 Hz')
legend('x', 'y', 'z')
hold off

% Save plot to file
save_path = folder_path + "image_velocity";
saveas(gcf,save_path, 'png')

figure(4)
plot(image_velocity_data_module1(:,1),LineWidth=3.0, Color="#0072BD")
title('x Instantaneous Camera Velocity')
ylabel('Instantaneous Camera Velocity (mm/sec) LT^{-1}')
xlabel('Iteration at 10 Hz')

save_path = folder_path + "image_velocity_x";
saveas(gcf,save_path, 'png')

figure(4)
plot(image_velocity_data_module1(:,2),LineWidth=3.0, Color="#D95319")
title('y Instantaneous Camera Velocity')
ylabel('Instantaneous Camera Velocity (mm/sec) LT^{-1}')
xlabel('Iteration at 10 Hz')

save_path = folder_path + "image_velocity_y";
saveas(gcf,save_path, 'png')

figure(4)
plot(image_velocity_data_module1(:,3),LineWidth=3.0, Color="#EDB120")
title('z Instantaneous Camera Velocity')
ylabel('Instantaneous Camera Velocity (mm/sec) LT^{-1}')
xlabel('Iteration at 10 Hz')

save_path = folder_path + "image_velocity_z";
saveas(gcf,save_path, 'png')