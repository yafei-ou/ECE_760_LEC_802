function J0 = getJacobianPSM(q, varargin)
%getJacobianPSM Jacobian in world coordinate
%
%   J0 = getJacobianPSM(q, options) is the jacobian of PSM (6x6).
%
%   Options: {'geometric', 'rpy', 'eul'}
%      'geometric'    geometric Jacobian
%      'rpy'          analytical Jacobian in RPY angles
%      'eul'          analytical Jacobian in ZYZ Euler angles


p = inputParser;
defaultType = 'geometric';
validType = {'geometric', 'rpy', 'eul'};
checkInputType = @(x) any(validatestring(x, validType));
addOptional(p,'type', defaultType, checkInputType);
parse(p, varargin{:})

J = getGeometricJacobianPSM(q);
Tr = fkinePSM(q);

switch p.Results.type
    case 'geometric'
        Ta = eye(3);
    case 'rpy'
        [~,p,y] = rotationMatrixToRPY(Tr);

        % X is roll:
        Ta = [	
            sin(p), 0, 1;
            -cos(p)*sin(y), cos(y), 0;
            cos(p)*cos(y), sin(y), 0;
            ];
        
    case 'eul'
        % convert to ZYZ euler angles
%         phi = atan2(R(2,3), R(1,3));
%         psi = atan2(R(3,2), -R(3,1));
%         theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
        [phi, theta, ~] = rotationMatrixToEulerAngles(Tr);

        % Ta
        Ta = [0, -sin(phi), cos(phi)*sin(theta);
            0, cos(phi), sin(phi)*sin(theta);
            1, 0, cos(theta)];
end
if rcond(Ta) < eps
    error("Representational singularity")
end

J0 = blkdiag(eye(3,3), inv(Ta)) * J;

end

