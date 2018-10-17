% Nicholas Fajardo
% RBE 595 - Advanced Surgical Robotics

classdef Wrist
    
    % Inner and Outer tube diameter
    properties (GetAccess = public, SetAccess = immutable)
        InnerDiameter
        OuterDiameter
    end
    
    % Notches in the wrist
    properties (GetAccess = public, SetAccess = private)
        notches
        baseLength
    end
    
    
    methods
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
            
            obj.notches = [obj.notches, notch];
        end
        
        % Sets a base length if appicable
        function obj = setBaseLength(obj, baseLength)
            obj.baseLength = baseLength;
        end
        
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
            
            transMatrices = [transMatrices, tBase];
            
            % Populates the transMatrices list with the matrices
            for index = size(obj.notches):-1:1
                % Grabbing notch (assumes list order of tip to beginning)
                notch = obj.notches(1, index);
                
                % Converting actuator space to configuration space
                % neutral bending plane displacement from notch frame
                y = nbpLoc(notch);
                k = deltaL/(notch.Height * ((obj.InnerDiameter/2) + y) - deltaL * y);
                s = notch.Height/(1 + y * k);
                
                % Calculating reference frame with gathered variables
                T_Notch = [1, 0, 0, 0;
                           0, cos(k*s), -sin(k*s), (cos(k*s) - 1)/k;
                           0, sin(k*s),  cos(k*s), sin(k*s)/k;
                           0, 0, 0, 1];
                
                transMatrices = [transMatrices, T_Notch];
                
                if notch.distanceFromPrev ~= 0
                    % Appends a translation amongst the Z axis
                    T_ToNext = [1, 0, 0, 0;
                                0, 1, 0, 0;
                                0, 0, 1, notch.distanceFromPrev;
                                0, 0, 0, 1]
                    
                    transMatrices = [transMatrices, T_ToNext];
                end
            end
            
            disp("Total Frames for FwKin of wrist found: " + size(transMatrices))
        end
        
        function T_Matrix = fsTransMatrix(rotZ, curvature, length)
            T_Matrix = [cos(rotZ)*cos(curvature*length), -sin(rotZ), cos(rotZ)*sin(curvature*length), (cos(rotZ)*(1-cos(curvature*length)))/(curvature);
                        sin(rotZ)*cos(curvature*length),  cos(rotZ), sin(rotZ)*sin(curvature*length), (sin(rotZ)*(1-cos(curvature*length)))/(curvature);
                        -sin(curvature*length),                   0, cos(curvature*length),           (sin(curvature*length))/(curvature);
                        0, 0, 0, 1];
        end
        
        function y = nbpLoc(obj, notch)
            phi_o = 2 * acos((notch.width - (obj.OuterDiameter))/(obj.OuterDiameter/2));
            phi_i = 2 * acos((notch.width - (obj.InnerDiameter))/(obj.InnerDiameter/2));
            y_o = (4 * (outerDiameter/2) * ((sin(.5 * phi_o))^3))/(3 * (phi_o - sin(phi_o)));
            y_i = (4 * (innerDiameter/2) * ((sin(.5 * phi_i))^3))/(3 * (phi_i - sin(phi_i)));
            A_o = (((outerDiameter/2)^2) * (phi_o - sin(phi_o)))/2;
            A_i = (((innerDiameter/2)^2) * (phi_i - sin(phi_i)))/2;
            
            y = (y_o * A_o - y_i * A_i)/(A_o - A_i);
        end
    end
end

