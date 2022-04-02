function [r, p, y] = rotationMatrixToRPY(R)
%rotationMatrixToRPY Calculate RPY angles from rotation matrix
%
% rpy = rotationMatrixToRPY(R) is the RPY angles for the rotation matrix.
% R can be either 3x3 or 4x4


if R(2,3) < eps && R(3,3) < eps
    % if cos(p) = 0
    r = 0;
    p = atan2(R(1,3), R(3,3));
    y = atan2(R(2,1), R(2,2));
else
    % psi = X
    r = atan2(R(3,2), R(3,3));
    % theta = Y
    p = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    % phi = Z
    y = atan2(R(2,1), R(1,1));
end


end

