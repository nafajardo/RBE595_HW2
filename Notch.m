% Nicholas Fajardo
% RBE 595 - Advanced Surgical Robotics
% Notch can represent a notch in a wrist, that has four defining properties (Height, Width, Orientation, and distance to the next notch)

classdef Notch < handle
    
    properties (GetAccess = public, SetAccess = immutable)
        % Height is the height of a notch
        % Width is the width of a notch
        % Orientation is the direction the notch was cut. Default is zero, but has a range of [0, 2pi)
        % distanceFromPrev is the distance above the notch to the next notch
        
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

