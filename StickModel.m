% Nicholas Fajardo
% RBE 595 - Advanced Surgical Robotics
% StickModel Represents configurations for a dexterous robotic manipulator, where all notches in the system are cut in the same orientation. 

clc
clear all

OuterDiameter = 1.8;
InnerDiameter = 1.6;
nHeight = 1;
nWidth = 1.6;
nDist = 1;
orientation = 0;
notch1 = Notch(nHeight, nWidth, orientation, nDist);
notch2 = Notch(nHeight, nWidth, orientation, nDist);
notch3 = Notch(nHeight, nWidth, orientation, nDist);
notch4 = Notch(nHeight, nWidth, orientation, nDist);

wrist = Wrist(InnerDiameter, OuterDiameter);

wrist.addNotch(notch1);
wrist.addNotch(notch2);
wrist.addNotch(notch3);
wrist.addNotch(notch4);

d_to_rad = pi/180;

configurations = [0, 0, 0;
                  .1, 0, 0;
                  .5, 0, 0;
                  .5, 20 * d_to_rad, 0;
                  .5, 90 * d_to_rad, 0;
                  .5, 90 * d_to_rad, 5];

% disp("Max angle wrist can assume (radians): " + wrist.maxAngleHomogeneous)
disp("Max angle wrist can assume (degrees): " + int32(wrist.maxAngleHomogeneous * (1/d_to_rad)))

fig = figure;
for index = 1:1:size(configurations, 1)
    q = configurations(index, :);
    disp(" ");
    disp ("Configuration being assumed: " + mat2str(q));
    T_Matrices = wrist.FwKin(q);
    points = pointsExtraction(T_Matrices);

    fig;
    subplot(2, 3, index);
    axis equal;
    hold on;
    grid on;
    view([1, 1, 1])
    title("Robot configuration: [ delta L = " + q(1, 1) + " mm, alpha = " + int32(q(1, 2) * (1/d_to_rad)) + " degrees, tau = " + q(1, 3) + " mm].");
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    plot3(points(1, :), points(2, :), points(3, :));
    scatter3(points(1, 1), points(2, 1), points(3, 1), 'r', 'filled');
    scatter3(points(1, size(points, 2)), points(2, size(points, 2)), points(3, size(points, 2)), 'g', 'filled');
end

function points = pointsExtraction(T_Matrices)
    points = [];
    
    for index = 1:size(T_Matrices, 3)
        
        T_Matrix = T_Matrices(:, :, 1);
        newPoint = T_Matrices(1:3, 4, 1);
        if index ~= 1
            for innerIndex = 2:1:index
                T_Matrix = T_Matrix * T_Matrices(:, :, innerIndex);
            end
            newPoint = T_Matrix(1:3, 4);
        end
        points = [points, newPoint];
    end
end