% Nicholas Fajardo
% RBE 595 - Advanced Surgical Robotics

clc
try
    OuterDiameter = 1.8;
    InnerDiameter = 1.6;
    nHeight = 1;
    nWidth = 1.6;
    nDist = 1;
    notch1 = Notch(nHeight, nWidth, 0, nDist);
    notch2 = Notch(nHeight, nWidth, 0, nDist);
    notch3 = Notch(nHeight, nWidth, 0, nDist);
    notch4 = Notch(nHeight, nWidth, 0, nDist);

    wrist = Wrist(InnerDiameter, OuterDiameter);

    wrist.addNotch(notch1);
    wrist.addNotch(notch2);
    wrist.addNotch(notch3);
    wrist.addNotch(notch4);

    d_to_rad = 180/pi;

    configurations = [0, 0, 0;
                      1, 0, 0;
                      5, 0, 0;
                      5, 20 * d_to_rad, 0;
                      5, 90 * d_to_rad, 0;
                      5, 90 * d_to_rad, 5];

    points = []
    
    for index = 1:size(configurations, 1)
        q = configurations(index, :);
        T_Matrices = wrist.FwKin(q);
        points(:, :, index) = pointsExtraction(T_Matrices);
    end
    
    disp(points(:,:,2))
catch
    disp("Something Went Wrong");
end
clear all

function points = pointsExtraction(T_Matrices)
    points = [];
    
    for index = 1:size(T_Matrices, 3)
        newPoint = T_Matrices(1:3, 4, index);
        points = [points, newPoint];
    end
end