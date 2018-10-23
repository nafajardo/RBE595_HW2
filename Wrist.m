% Nicholas Fajardo
% RBE 595 - Advanced Surgical Robotics
% Wrist object is able to represent a notched wrist
% One can create this class and call FwKin and FwKin 2 to get transformation matrices to represent certain configurations of a custom wrist
% See Notch help for details on what a Notch can represent

classdef Wrist < handle
    %% Wrist Properties
    % Inner and Outer tube diameter
    properties (GetAccess = public, SetAccess = immutable)
        % InnerDiameter represents the inner diameter of the tube
        % OuterDiameter represents the outer diameter of the tube
        InnerDiameter
        OuterDiameter
    end
    
    properties (GetAccess = public, SetAccess = private)
        % notches represents the notches in the wrist (ordered from tip to beginning by default)
        % baseLength represents the length from the base of the dexterous wrist to the first notch
        notches
        baseLength
    end
    
    methods
        %% Class Construction
        % Constructor
        function obj = Wrist(innerDiameter, outerDiameter)
            obj.InnerDiameter = innerDiameter;
            obj.OuterDiameter = outerDiameter;
            obj.baseLength = 0;
            obj.notches = [];
        end
        
        % Appends a notch to the wrist design
        % Calculations depend on notch order being top down (tip to start)
        function obj = addNotch(obj, notch)
            if class(notch) ~= "Notch"
                error("Notch input must be an instance of a Notch class")
            end
            
            if size(obj.notches) == 0
                obj.notches = [notch];
            else
                obj.notches = [obj.notches, notch];
            end
        end
        
        % Sets a base length if appicable
        function obj = setBaseLength(obj, baseLength)
            obj.baseLength = baseLength;
        end
        
        %% FwKin
        % Calculates the transformation matrices needed for the wrist using
        % Frenet Serret frames, and assumes all notches are positioned the
        % same (all notches have an orientation = 0 radians)
        function transMatrices = FwKin(obj, q)
            % Extracts the actuator variables
            deltaL = q(1, 1);
            alpha = q(1, 2);
            tau = q(1, 3);
            
            transMatrices = [];
            
            % Calculates the transformation matrix for the base translation
            tBase = [cos(alpha), -sin(alpha), 0, 0;
                     sin(alpha),  cos(alpha), 0, 0;
                     0, 0, 1, obj.baseLength + tau;
                     0, 0, 0, 1];
            
            transMatrices(:, :, 1) = tBase;
            
            % Populates the transMatrices list with the matrices
            for index = size(obj.notches, 2):-1:1
                % Grabbing notch (assumes list order of tip to beginning)
                notch = obj.notches(1, index);
                
                % Converting actuator space to configuration space
                % neutral bending plane displacement from notch frame
                y = obj.nbpLoc(notch);
                k = deltaL/(notch.Height * ((obj.InnerDiameter/2) + y) - deltaL * y);
                s = notch.Height/(1 + y * k);
                
                % Printing warnings if strain and/or max angle is surpassed
                % NOTE: material is assumed to be NiTi
                strain = obj.calcMaxStrain(y, k, deltaL);
                disp("Max Strain induced on notch " + index + " by tendon displacement: " + deltaL + ": " + int32(strain * 100) + "%.");
                if strain > 0.08
                    disp ("Calculated strain for tendon displacement of " + deltaL + " exceeds maximum strain limit of 8%!")
                end
                
                currentAngle = (notch.Height/(1 + (y * k))) * k;
                maxAngle = (notch.Height/((obj.OuterDiameter/2) + y));
                if abs(currentAngle) > abs(maxAngle)
                    disp("Current tendon displacement (" + currentAngle + ") surpasses maximum angle (" + maxAngle + ")!");
                end
                
                T_Notch = zeros(4, 4);
                % Handles the case when there is no curvature
                if(k ~= 0)
                    % Calculating reference frame with gathered variables
                    T_Notch = [1, 0, 0, 0;
                               0, cos(k*s), -sin(k*s), (cos(k*s) - 1)/k;
                               0, sin(k*s),  cos(k*s), sin(k*s)/k;
                               0, 0, 0, 1];
                else
                    % Calculating reference frame with gathered variables
                    T_Notch = [1, 0, 0, 0;
                               0, cos(k*s), -sin(k*s), 0;
                               0, sin(k*s),  cos(k*s), s;
                               0, 0, 0, 1];
                end
                
                transMatrices(:, :, size(transMatrices, 3) + 1) = T_Notch;
                
                if notch.distanceFromPrev ~= 0
                    % Appends a translation amongst the Z axis
                    T_ToNext = [1, 0, 0, 0;
                                0, 1, 0, 0;
                                0, 0, 1, notch.distanceFromPrev;
                                0, 0, 0, 1];
                    
                    transMatrices(:, :, size(transMatrices, 3) + 1) =  T_ToNext;
                end
            end
            
            % disp("Total Frames for FwKin of wrist found: " + size(transMatrices, 3))
        end
        
        %% FwKin2
        % Calculates the transformation matrices needed for the wrist using
        % Frenet Serret frames, and allows for different orientations in
        % notch design (notch orientation is in subset of R space (0, 2pi]
        % ASSUMES THE DELTA_L VECTOR IS ORDERED FROM 0 -> 2pi
        function transMatrices = FwKin2(obj, tl, q)
            % Extracts the actuator variables
            deltaL_Vec = tl;
            alpha = q(1, 1);
            tau = q(1, 2);
            
            % Pairing the variance and deltaL_Vec
            variance = obj.notchVariance();
            
            tuples = [variance; deltaL_Vec];
            
            transMatrices = [];
            
            % Calculates the transformation matrix for the base translation
            tBase = [cos(alpha), -sin(alpha), 0, 0;
                     sin(alpha),  cos(alpha), 0, 0;
                     0, 0, 1, obj.baseLength + tau;
                     0, 0, 0, 1];
            
            transMatrices(:, :, 1) = tBase;
            
            previousNotchOrientation = 0;
            
            % Populates the transMatrices list with the matrices
            for index = size(obj.notches, 2):-1:1
                % Grabbing notch (assumes list order of tip to beginning)
                notch = obj.notches(1, index);
                
                % Appends a Transfomation Matrix that handles different
                % notch orientations
                if ~(notch.Orientation == previousNotchOrientation)
                    deltaDegree = notch.Orientation - previousNotchOrientation;
                    previousNotchOrientation = notch.Orientation;
                    transMatrices(:, :, size(transMatrices, 3) + 1) = [cos(deltaDegree), -sin(deltaDegree), 0, 0;
                                                                       sin(deltaDegree),  cos(deltaDegree), 0, 0;
                                                                       0, 0, 1, 0;
                                                                       0, 0, 0, 1];
                end
                
                indexDL = find(tuples(1, :) == notch.Orientation);
                deltaL = tuples(2, indexDL);
                
                % Converting actuator space to configuration space
                % neutral bending plane displacement from notch frame
                y = obj.nbpLoc(notch);
                k = deltaL/(notch.Height * ((obj.InnerDiameter/2) + y) - deltaL * y);
                s = notch.Height/(1 + y * k);
                
                
                % Printing warnings if strain and/or max angle is surpassed
                % NOTE: material is assumed to be NiTi
                strain = obj.calcMaxStrain(y, k, deltaL);
                disp("Max Strain induced on notch " + index + " by tendon displacement: " + deltaL + ": " + int32(strain * 100) + "%.");
                if strain > 0.08
                    disp ("Calculated strain for tendon displacement of " + deltaL + " exceeds maximum strain limit of 8%!")
                end
                
                T_Notch = zeros(4, 4);
                % Handles the case when there is no curvature
                if(k ~= 0)
                    % Calculating reference frame with gathered variables
                    T_Notch = [1, 0, 0, 0;
                               0, cos(k*s), -sin(k*s), (cos(k*s) - 1)/k;
                               0, sin(k*s),  cos(k*s), sin(k*s)/k;
                               0, 0, 0, 1];
                else
                    % Calculating reference frame with gathered variables
                    T_Notch = [1, 0, 0, 0;
                               0, cos(k*s), -sin(k*s), 0;
                               0, sin(k*s),  cos(k*s), s;
                               0, 0, 0, 1];
                end
                
                transMatrices(:, :, size(transMatrices, 3) + 1) = T_Notch;
                
                if notch.distanceFromPrev ~= 0
                    % Appends a translation amongst the Z axis
                    T_ToNext = [1, 0, 0, 0;
                                0, 1, 0, 0;
                                0, 0, 1, notch.distanceFromPrev;
                                0, 0, 0, 1];
                    
                    transMatrices(:, :, size(transMatrices, 3) + 1) =  T_ToNext;
                end
            end
            
            % disp("Total Frames for FwKin of wrist found: " + size(transMatrices, 3))
        end
        
        %% Helper Functions
        % Returns the max angle a wrist can form with homogeneous wrist
        function theta = maxAngleHomogeneous(obj)
            sampleNotch = obj.notches(1, 1);
            h = sampleNotch.Height;
            theta = size(obj.notches, 2)*(h/((obj.OuterDiameter/2) + obj.nbpLoc(sampleNotch)));
        end
        
        function T_Matrix = fsTransMatrix(rotZ, curvature, length)
            T_Matrix = [cos(rotZ)*cos(curvature*length), -sin(rotZ), cos(rotZ)*sin(curvature*length), (cos(rotZ)*(1-cos(curvature*length)))/(curvature);
                        sin(rotZ)*cos(curvature*length),  cos(rotZ), sin(rotZ)*sin(curvature*length), (sin(rotZ)*(1-cos(curvature*length)))/(curvature);
                        -sin(curvature*length),                   0, cos(curvature*length),           (sin(curvature*length))/(curvature);
                        0, 0, 0, 1];
        end
        
        function y = nbpLoc(obj, notch)
            phi_o = 2 * acos((notch.Width - (obj.OuterDiameter))/(obj.OuterDiameter/2));
            phi_i = 2 * acos((notch.Width - (obj.InnerDiameter))/(obj.InnerDiameter/2));
            y_o = (4 * (obj.OuterDiameter/2) * ((sin(.5 * phi_o))^3))/(3 * (phi_o - sin(phi_o)));
            y_i = (4 * (obj.InnerDiameter/2) * ((sin(.5 * phi_i))^3))/(3 * (phi_i - sin(phi_i)));
            A_o = (((obj.OuterDiameter/2)^2) * (phi_o - sin(phi_o)))/2;
            A_i = (((obj.InnerDiameter/2)^2) * (phi_i - sin(phi_i)))/2;
            
            y = (y_o * A_o - y_i * A_i)/(A_o - A_i);
        end
        
        function strain = calcMaxStrain(obj, y, k, deltaL)
            strain_Inner = (k * ((obj.InnerDiameter/2) - y))/(1 + (y * k));
            strain_Outer = (k * ((obj.OuterDiameter/2) - y))/(1 + (y * k));
            
            strain = max(strain_Inner, strain_Outer);
        end
        
        % Returns a sorted list of notch orientation values in the wrist (no repeats)
        function variance = notchVariance(obj)
            variance = [];
            for notchIndex = 1:size(obj.notches, 2)
                orientation = obj.notches(1, notchIndex).Orientation;
                if ~any(variance(:) == orientation)
                    variance = [variance, orientation];
                end
            end
            
            variance = sort(variance);
        end
    end
end

