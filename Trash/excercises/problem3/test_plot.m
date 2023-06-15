%% Example distance and yaw data (by GPT-4)
x = [0, 1, 2, 3, 4, 5];  % x distances
y = [0, 2, 4, 6, 8, 10]; % y distances
yaw = [0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4]; % yaw angles in radians

%%% personal additions
figure(1); clf;

% Plotting the distance traveled as a trajectory
plot(x, y, '-');
xlabel('X Distance');
ylabel('Y Distance');
title('Distance Traveled');

hold on; % Hold the plot to add line segments

% Adding line segments for orientation based on yaw angle
scale = 0.2; % Scale factor for line segment length

for i = 1:numel(x)
    x_end   = x(i) + scale*(cos(yaw(i))); % x-coordinate of line segment endpoint
    y_end   = y(i) + scale*(sin(yaw(i))); % y-coordinate of line segment endpoint
    
    plot([x_start, x_end], [y_start, y_end], 'r-'); % Plotting line segment
end

hold off; % Release the hold on the plot

%% Example distance and yaw data (my modification)
x = [0, 1, 2, 3, 4, 5];  % x distances
y = [0, 2, 4, 6, 8, 10]; % y distances
yaw = [0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4]; % yaw angles in radians

figure(2); clf;
% Plotting the distance traveled as a trajectory
plot(x, y, '-');
xlabel('X Distance');
ylabel('Y Distance');
title('Distance Traveled');

hold on; % Hold the plot to add line segments

% Adding line segments for orientation based on yaw angle
scale = 0.4; % Scale factor for line segment length

for i = 1:numel(x)
    x_start = x(i) - scale*(cos(yaw(i)))/2; % x-coordinate of line segment startpoint
    x_end   = x(i) + scale*(cos(yaw(i)))/2; % x-coordinate of line segment endpoint
    y_start = y(i) - scale*(sin(yaw(i)))/2; % y-coordinate of line segment startpoint
    y_end   = y(i) + scale*(sin(yaw(i)))/2; % y-coordinate of line segment endpoint
    
    plot([x_start, x_end], [y_start, y_end], 'r-','LineWidth',4); % Plotting line segment
end

hold off; % Release the hold on the plot