% Nicholas Fajardo
% RBE 595 - Advanced Surgical Robotics

classdef Wrist
    
    % Inner and Outer tube diameter
    % Can only be set on construction, but can be read by all
    properties (GetAccess = public, SetAccess = immutable)
        InnerDiameter
        OuterDiameter
    end
    
    % Notches in the wrist
    % Can only be set from within the class, but can be read by all
    properties (GetAccess = public, SetAccess = private)
        notches
        baseLength
    end
    
    
    methods
        % Constructor
        function obj = Wrist(innerDiameter, outerDiameter)
            obj.InnerDiameter = innerDiameter;
            obj.OuterDiameter = outerDiameter;
        end
        
        function obj = addNotch(obj, notch)
            %addNotch takes in a notch and adds it to the wrist
            if class(notch) ~= "Notch"
                error("Notch input must be an instance of a Notch class")
            end
            obj.notches = [obj.notches, notch];
        end
        
        function obj = setBaseLength(obj, baseLength)
            obj.baseLength = baseLength;
        end
        
        function obj = FwKin(obj, q)
            % FwKin calculates the set of forward kinematics transformation
            % matrices needed to represent the wrist
            % q is the set of actuator variables
            deltaL = q(1, 1);
            alpha = q(1, 2);
            tau = q(1, 3);
            
            transMatrices = [];
            
            % Add the transformation matrix of a base length translation
            % among the Z
            tBase = [0, 0, 0, 0;
                     0, 0, 0, 0;
                     0, 0, 0, obj.baseLength;
                     0, 0, 0, 1];
            
            transMatrices = [transMatrices, tBase];
        end
    end
end

