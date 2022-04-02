function [omega, vel] = velocityIteration(transMat,omegaPrev, velPrev, type, jointVar)

rotMat = transMat(1:3, 1:3).';
posOrigin = transMat(1:3, 4);


if type == 'R'
    omega = rotMat*omegaPrev + [0;0;jointVar];
    vel = rotMat*(velPrev + cross(omegaPrev, posOrigin));
elseif type == 'P'
    omega = rotMat*omegaPrev;
    vel = rotMat*(velPrev + cross(omegaPrev, posOrigin)) + [0;0;jointVar];


end

end