function [phi,theta,psi] = rotationMatrixToEulerAngles(R)
%ROTATIONMATRIXTOEULERANGLES Summary of this function goes here
%   Detailed explanation goes here
    phi = atan2(R(2,3), R(1,3));
    psi = atan2(R(3,2), -R(3,1));
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
end

