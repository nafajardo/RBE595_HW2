function T_Matrix = fsTransMatrix(rotZ, curvature, length)
            % fsTransMatrix takes in the variables needed for a Frenet
            % Serret Transformation matrix, and returns the Homogeneous
            % Transformation Matrix using the given parameters
            T_Matrix = [cos(rotZ)*cos(curvature*length), -sin(rotZ), cos(rotZ)*sin(curvature*length), (cos(rotZ)*(1-cos(curvature*length)))/(curvature);
                        sin(rotZ)*cos(curvature*length),  cos(rotZ), sin(rotZ)*sin(curvature*length), (sin(rotZ)*(1-cos(curvature*length)))/(curvature);
                        -sin(curvature*length),                   0, cos(curvature*length),           (sin(curvature*length))/(curvature);
                        0, 0, 0, 1];
        end