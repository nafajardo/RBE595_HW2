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
    end
end

