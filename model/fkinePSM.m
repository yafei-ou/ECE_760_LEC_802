function T = fkinePSM(q)
%FKINEPSM Forward kinematics of PSM
%   T = fkinePSM(q) is the 4x4 transformation matrix of forward kinematics
%   for PSM.

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);

T = [cos(q2)*cos(q5)*sin(q1)*sin(q6) - cos(q1)*cos(q4)*cos(q6) + cos(q6)*sin(q1)*sin(q2)*sin(q4) - cos(q1)*sin(q4)*sin(q5)*sin(q6) - cos(q4)*sin(q1)*sin(q2)*sin(q5)*sin(q6),...
    cos(q1)*cos(q4)*sin(q6) + cos(q2)*cos(q5)*cos(q6)*sin(q1) - cos(q1)*cos(q6)*sin(q4)*sin(q5) - sin(q1)*sin(q2)*sin(q4)*sin(q6) - cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q5),...
    - cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q5) - cos(q4)*cos(q5)*sin(q1)*sin(q2),...
    q3*cos(q2)*sin(q1) - (39*cos(q2)*sin(q1))/2500 + (91*cos(q2)*cos(q5)*sin(q1))/10000 - (91*cos(q1)*sin(q4)*sin(q5))/10000 - (91*cos(q4)*sin(q1)*sin(q2)*sin(q5))/10000;

    cos(q2)*cos(q6)*sin(q4) - cos(q5)*sin(q2)*sin(q6) - cos(q2)*cos(q4)*sin(q5)*sin(q6),...
    - cos(q5)*cos(q6)*sin(q2) - cos(q2)*sin(q4)*sin(q6) - cos(q2)*cos(q4)*cos(q6)*sin(q5),...
    sin(q2)*sin(q5) - cos(q2)*cos(q4)*cos(q5),...
    (39*sin(q2))/2500 - (91*cos(q5)*sin(q2))/10000 - q3*sin(q2) - (91*cos(q2)*cos(q4)*sin(q5))/10000;
    
    cos(q1)*cos(q4)*sin(q2)*sin(q5)*sin(q6) - cos(q1)*cos(q2)*cos(q5)*sin(q6) - cos(q1)*cos(q6)*sin(q2)*sin(q4) - sin(q1)*sin(q4)*sin(q5)*sin(q6) - cos(q4)*cos(q6)*sin(q1),...
    cos(q4)*sin(q1)*sin(q6) - cos(q1)*cos(q2)*cos(q5)*cos(q6) + cos(q1)*sin(q2)*sin(q4)*sin(q6) - cos(q6)*sin(q1)*sin(q4)*sin(q5) + cos(q1)*cos(q4)*cos(q6)*sin(q2)*sin(q5),...
    cos(q1)*cos(q2)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q5)*sin(q2),...
    (39*cos(q1)*cos(q2))/2500 - (91*sin(q1)*sin(q4)*sin(q5))/10000 - q3*cos(q1)*cos(q2) - (91*cos(q1)*cos(q2)*cos(q5))/10000 + (91*cos(q1)*cos(q4)*sin(q2)*sin(q5))/10000;
    
    0, 0, 0, 1];
end

