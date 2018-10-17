% Nicholas Fajardo
% RBE 595 - Advanced Surgical Robotics

classdef Notch < handle
    
    properties (GetAccess = public, SetAccess = immutable)
        Height
        Width
        Orientation
        distanceFromPrev
    end
    
    methods
        function obj = Notch(height, width, orientation, distanceFromPrev)
            obj.Height = height;
            obj.Width = width;
            obj.Orientation = orientation;
            obj.distanceFromPrev = distanceFromPrev;
        end
    end
end

