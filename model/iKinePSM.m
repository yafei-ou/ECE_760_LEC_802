function [q, c] = iKinePSM(transMat, useCorrection)
%iKinePSM Inverse kinematics of PSM.
%
%   Geometric method is used. Based on the work by Filippo Mantovan:
%   https://github.com/Filippo39/IKsolution_dvrk, with improvements and bug
%   fixes.
%
%   [q, c] = iKinePSM(transMat, useCorrection)
% 
%   q is a 2x6 vector. Each row is a possible solution of [theta1, theta2,
%   d3 theta4 theta5 theta6]. 
%
%   c is a logical flag indicating whether correction to theta4 and theta5 
%   was used when calculating the results.

q = zeros(2,6);

% RCM position is always zero.
RCM = [0;0;0];

% DH param a(6-1) = 0.0091
offsetA6 = 0.0091;

% Use Tr from workspace calculated from forward kinematics.
transMatBaseToFrame6 = transMat;

% Position of End-effector
EE = transMatBaseToFrame6(1:3,4);


% Move End-effector along a vector to get the origin of Frame 5.
vectEEAxisZ = transMatBaseToFrame6(1:3,3);
vectRCMToEE = EE - RCM;
vectPlaneNormalRCMToEEAxisZ = cross(vectRCMToEE, vectEEAxisZ);
vectPlaneNormalEEAxisXY = vectEEAxisZ;

vectMoveDirection = cross(vectPlaneNormalEEAxisXY, vectPlaneNormalRCMToEEAxisZ);
vectMoveDirection = vectMoveDirection./norm(vectMoveDirection);
c = false;


for i=1:2
    originFrame5 = EE - offsetA6 .* vectMoveDirection;

    % Calculate first 3 joint values.
    d3 = norm(originFrame5);

    % Depending on whether the origin of frame 5 is above or below the
    % RCM, calculate real d3. We can do it in this simple way as we
    % have the first and second joint limits, and both of them cannot make
    % the robot flip. So d3 is the only factor deciding whether frame
    % 5 is below or above RCM.
    if originFrame5(3) <= 0
        d3 = d3 + 0.0156;
    else
        d3 = 0.0156 - d3;
    end

    theta1 = - atan(originFrame5(1)/originFrame5(3));
    theta2 = - asin(originFrame5(2)/(d3 - 0.0156));

    % Calculate theta4.
    transPosBaseToFrame4 = originFrame5 - RCM; % origin of Frames 4 and 5 are the same
    vectRCMToFrame4 = transPosBaseToFrame4./norm(transPosBaseToFrame4);
    vectPlaneNormalRCM2Frame4AxisXZ = vectPlaneNormalRCMToEEAxisZ; % 2 Planes are the same
    vectFrame4AxisYRef = cross([0;1;0], vectRCMToFrame4);
    [theta4, vectFrame4AxisZ] = angleOfVectors(vectPlaneNormalRCM2Frame4AxisXZ, vectFrame4AxisYRef);

    if dot(transPosBaseToFrame4, vectFrame4AxisZ) >= 0
        theta4 = -theta4;
    end

    % Calculate theta5
    vectFrame5ToFrame6 = EE - originFrame5;
    [theta5, vectDirectionOfAngle5] = angleOfVectors(vectFrame5ToFrame6, -transPosBaseToFrame4);
    theta5 = pi - theta5;

    if abs(theta5) > pi/2
        theta5 = - theta5;
        if theta4 > 0
            theta4 = theta4 - pi;
        elseif theta4 < 0
            theta4 = pi + theta4;
        end
    end

    % Calculate theta6
    vectFrame6AxisY = transMatBaseToFrame6(1:3, 2);
    [theta6, vectDirectionOfAngle6] = angleOfVectors(-vectFrame5ToFrame6, vectFrame6AxisY);
    theta6 = pi - theta6;
    if dot(vectEEAxisZ, vectDirectionOfAngle6) > 0
        theta6 = - theta6;
    end

    % Correct theta4 and theta5 using fkine.
    direction = dot(vectPlaneNormalRCM2Frame4AxisXZ, vectDirectionOfAngle5);
    % distanceRCMToOriginFrame5 = norm(originFrame5);
    isCorrectionNeeded = abs(direction) < 1e-4  || abs(EE(3)) < 0.01;
    c = isCorrectionNeeded;
    if isCorrectionNeeded && useCorrection


        if originFrame5(3) > 0
            listThetaTest = [
                - theta4 - pi, pi + theta5;
                - theta4 - pi, pi - theta5;
                - theta4, pi + theta5;
                - theta4, pi - theta5;
            ];
%             transMatTest = 
% [cos(q2)*cos(q5)*sin(q1)*sin(q6) - 1.0*cos(q1)*cos(q4)*cos(q6) + cos(q6)*sin(q1)*sin(q2)*sin(q4) - 1.0*cos(q1)*sin(q4)*sin(q5)*sin(q6) - 1.0*cos(q4)*sin(q1)*sin(q2)*sin(q5)*sin(q6),    cos(q1)*cos(q4)*sin(q6) + cos(q2)*cos(q5)*cos(q6)*sin(q1) - 1.0*cos(q1)*cos(q6)*sin(q4)*sin(q5) - 1.0*sin(q1)*sin(q2)*sin(q4)*sin(q6) - 1.0*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q5), - 1.0*cos(q1)*cos(q5)*sin(q4) - 1.0*cos(q2)*sin(q1)*sin(q5) - 1.0*cos(q4)*cos(q5)*sin(q1)*sin(q2),     q3*cos(q2)*sin(q1) - 0.0156*cos(q2)*sin(q1) + 0.0091*cos(q2)*cos(q5)*sin(q1) - 0.0091*cos(q1)*sin(q4)*sin(q5) - 0.0091*cos(q4)*sin(q1)*sin(q2)*sin(q5);
%                                                                                             cos(q2)*cos(q6)*sin(q4) - 1.0*cos(q5)*sin(q2)*sin(q6) - 1.0*cos(q2)*cos(q4)*sin(q5)*sin(q6),                                                                                   - 1.0*cos(q5)*cos(q6)*sin(q2) - 1.0*cos(q2)*sin(q4)*sin(q6) - 1.0*cos(q2)*cos(q4)*cos(q6)*sin(q5),                                                     sin(q2)*sin(q5) - 1.0*cos(q2)*cos(q4)*cos(q5),                                                                  0.0156*sin(q2) - 0.0091*cos(q5)*sin(q2) - 1.0*q3*sin(q2) - 0.0091*cos(q2)*cos(q4)*sin(q5);
% cos(q1)*cos(q4)*sin(q2)*sin(q5)*sin(q6) - 1.0*cos(q1)*cos(q2)*cos(q5)*sin(q6) - 1.0*cos(q1)*cos(q6)*sin(q2)*sin(q4) - 1.0*sin(q1)*sin(q4)*sin(q5)*sin(q6) - 1.0*cos(q4)*cos(q6)*sin(q1),     cos(q4)*sin(q1)*sin(q6) - 1.0*cos(q1)*cos(q2)*cos(q5)*cos(q6) + cos(q1)*sin(q2)*sin(q4)*sin(q6) - 1.0*cos(q6)*sin(q1)*sin(q4)*sin(q5) + cos(q1)*cos(q4)*cos(q6)*sin(q2)*sin(q5),           cos(q1)*cos(q2)*sin(q5) - 1.0*cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q5)*sin(q2), 0.0156*cos(q1)*cos(q2) - 0.0091*sin(q1)*sin(q4)*sin(q5) - 1.0*q3*cos(q1)*cos(q2) - 0.0091*cos(q1)*cos(q2)*cos(q5) + 0.0091*cos(q1)*cos(q4)*sin(q2)*sin(q5)]

        else
            listThetaTest = [
                theta4, - theta5;
                theta4, theta5;
                theta4 - pi, theta5;
                theta4 - pi, - theta5;
            ];
        end
        for j=1:4
            theta4 = listThetaTest(j,1);
            theta5 = listThetaTest(j,2);
            transMatTestAxisX = [cos(theta2)*cos(theta5)*sin(theta1)*sin(theta6) - 1.0*cos(theta1)*cos(theta4)*cos(theta6) + cos(theta6)*sin(theta1)*sin(theta2)*sin(theta4) - 1.0*cos(theta1)*sin(theta4)*sin(theta5)*sin(theta6) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta5)*sin(theta6);
                cos(theta2)*cos(theta6)*sin(theta4) - 1.0*cos(theta5)*sin(theta2)*sin(theta6) - 1.0*cos(theta2)*cos(theta4)*sin(theta5)*sin(theta6);
                cos(theta1)*cos(theta4)*sin(theta2)*sin(theta5)*sin(theta6) - 1.0*cos(theta1)*cos(theta2)*cos(theta5)*sin(theta6) - 1.0*cos(theta1)*cos(theta6)*sin(theta2)*sin(theta4) - 1.0*sin(theta1)*sin(theta4)*sin(theta5)*sin(theta6) - 1.0*cos(theta4)*cos(theta6)*sin(theta1)];
            transMatTestAxisY = [cos(theta1)*cos(theta4)*sin(theta6) + cos(theta2)*cos(theta5)*cos(theta6)*sin(theta1) - 1.0*cos(theta1)*cos(theta6)*sin(theta4)*sin(theta5) - 1.0*sin(theta1)*sin(theta2)*sin(theta4)*sin(theta6) - 1.0*cos(theta4)*cos(theta6)*sin(theta1)*sin(theta2)*sin(theta5);
                - 1.0*cos(theta5)*cos(theta6)*sin(theta2) - 1.0*cos(theta2)*sin(theta4)*sin(theta6) - 1.0*cos(theta2)*cos(theta4)*cos(theta6)*sin(theta5);
                cos(theta4)*sin(theta1)*sin(theta6) - 1.0*cos(theta1)*cos(theta2)*cos(theta5)*cos(theta6) + cos(theta1)*sin(theta2)*sin(theta4)*sin(theta6) - 1.0*cos(theta6)*sin(theta1)*sin(theta4)*sin(theta5) + cos(theta1)*cos(theta4)*cos(theta6)*sin(theta2)*sin(theta5)];
            error = immse([transMatTestAxisX, transMatTestAxisY], transMat(1:3,1:2));
            if error < 1e-6
                break
            else
                continue
            end
        end
        if abs(theta4) > pi
            theta4 = sign(theta4)*(abs(theta4)-2*pi);
        end

        if abs(theta5) > pi
            theta5 = sign(theta5)*(abs(theta5)-2*pi);
        end
        
    else
        isOppositeDirection = direction <= 0;
        if isOppositeDirection
            theta5 = - theta5;
        end

        if isOppositeDirection && theta5 < - pi/2
            theta5 = - theta5;
        end
    end

    q(i,:) = [theta1, theta2, d3 theta4 theta5 theta6];
    offsetA6 = -offsetA6;
end
end