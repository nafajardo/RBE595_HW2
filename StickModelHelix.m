% Nicholas Fajardo
% RBE 595 - Advanced Surgical Robotics
% StickModelHelix represents a Helix shape for a robotic manipulator, where each of the 8 notches varies in orientation by a magnitude of 90 degrees

clc
clear all

OuterDiameter = 4;
InnerDiameter = 3;
nHeight = 5;
nWidth = 3;
nDist = 2;
notch1 = Notch(nHeight, nWidth, 0, nDist);
notch2 = Notch(nHeight, nWidth, pi/2, nDist);
notch3 = Notch(nHeight, nWidth, pi, nDist);
notch4 = Notch(nHeight, nWidth, 3*(pi/2), nDist);
notch5 = Notch(nHeight, nWidth, 0, nDist);
notch6 = Notch(nHeight, nWidth, pi/2, nDist);
notch7 = Notch(nHeight, nWidth, pi, nDist);
notch8 = Notch(nHeight, nWidth, 3*(pi/2), nDist);

wrist = Wrist(InnerDiameter, OuterDiameter);

wrist.addNotch(notch1);
wrist.addNotch(notch2);
wrist.addNotch(notch3);
wrist.addNotch(notch4);
wrist.addNotch(notch5);
wrist.addNotch(notch6);
wrist.addNotch(notch7);
wrist.addNotch(notch8);

d_to_rad = pi/180;

tendon_displacements = [0, 0, 0, 0;
                        0.1, 0.1, 0.1, 0.1;
                        0.8, 0.8, 0.8, 0.8;
                        1, 1, 1, 1;
                        2, 2, 2, 2;
                        3, 3, 3, 3;];

configurations = [0, 0;
                  0, 0;
                  0, 0;
                  0, 0;
                  0, 0;
                  0, 0];

% disp("Max angle wrist can assume (radians): " + wrist.maxAngleHomogeneous)
disp("Max angle wrist can assume (degrees): " + int32(wrist.maxAngleHomogeneous * (1/d_to_rad)))

fig = figure;
for index = 1:1:size(configurations, 1)
    tl = tendon_displacements(index, :);
    q = configurations(index, :);
    disp(" ");
    disp ("Configuration being assumed: deltaL vector " + mat2str(tl) + " alpha and tau " + mat2str(q));
    T_Matrices = wrist.FwKin2(tl, q);
    
    
    points = pointsExtraction(T_Matrices);

    fig;
    subplot(2, 3, index);
    axis equal;
    hold on;
    grid on;
    view([1, 1, 1])
    title("Robot configuration: [delta L = " + mat2str(tl(1, :)) + " mm, alpha = " + int32(q(1, 1) * (1/d_to_rad)) + " degrees, tau = " + q(1, 2) + " mm].");
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