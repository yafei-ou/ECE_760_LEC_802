classdef RobotDynamics < handle
    %RobotDynamics A class that calculates the dynamics of a robot with
    %less than 7 DoFs, in symbolic form.
    %   A Matlab implementation of the dynamics calculation method used in
    %   [1]	Y. Wang, R. Gondokaryono, A. Munawar, and G. S. Fischer, “A 
    %   convex optimization-based dynamic model identification package for 
    %   the da Vinci research kit," IEEE Robot. Autom. Lett., vol. 4, no. 
    %   4, pp. 3657–3664, 2019.
    %
    %   Code based on: https://github.com/WPI-AIM/dvrk_dynamics_identification

    
    properties
        isSymbolic
        tau
        M
        C
        G
        Y
        Yb
        baseParams
        
        n
        m
        r
        l % COM; m*r=l
        I
        L
        velocityLink
        velocityCOM
        wLink
        
        baryParams
        stdDynParams
        
        rtbRobot
        q_t
        dq_t
        ddq_t
        diff_q
        diff_dq
        q
        dq
        ddq
        gravity
        k
        u
        
    end
    
    methods
        function robot = RobotDynamics(rtbRobot)
            %RobotDynamics Construct an instance of this class
            %   Detailed explanation goes here
            
            % Robot DOF
            robot.n = rtbRobot.n;
            robot.rtbRobot = rtbRobot;
            
            % assume at most 7 joints (because I don't know how to generate
            % a list of symfun automatically)
            syms theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t) theta7(t)
            syms theta_dot_1(t) theta_dot_2(t) theta_dot_3(t) theta_dot_4(t) theta_dot_5(t) theta_dot_6(t) theta_dot_7(t)
            syms theta_ddot_1(t) theta_ddot_2(t) theta_ddot_3(t) theta_ddot_4(t) theta_ddot_5(t) theta_ddot_6(t) theta_ddot_7(t)
            syms g
            
            robot.q_t = [theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t)];
            robot.dq_t = [theta_dot_1(t) theta_dot_2(t) theta_dot_3(t) theta_dot_4(t) theta_dot_5(t) theta_dot_6(t)];
            robot.ddq_t = [theta_ddot_1(t) theta_ddot_2(t) theta_ddot_3(t) theta_ddot_4(t) theta_ddot_5(t) theta_ddot_6(t)];
            
            robot.q_t = robot.q_t(1,1:robot.n);
            robot.dq_t = robot.dq_t(1,1:robot.n);
            robot.ddq_t = robot.ddq_t(1,1:robot.n);

            robot.q = sym("q", [1,robot.n]);
            robot.dq = sym("dq", [1,robot.n]);
            robot.ddq = sym("ddq", [1,robot.n]);
            
            robot.diff_q = diff(robot.q_t, t);
            robot.diff_dq = diff(robot.dq_t, t);
            
            robot.gravity = [0,0,-g];

            
            robot.m = sym(zeros(robot.n,1));
            robot.l = sym(zeros(robot.n,3));
            robot.r = sym(zeros(robot.n,3));
            
            robot.I = sym(zeros(robot.n,6));
            robot.L = sym(zeros(robot.n,6));
            
            for num=1:robot.n
                m = rtbRobot.links(num).m;
                r = rtbRobot.links(num).r;
                I = rtbRobot.links(num).I;
                L = I2L(I,m,r);
                robot.m(num,:) = m;
                robot.r(num,:) = r;
                robot.l(num,:) = m*r;
                robot.I(num,:) = [I(1,1),I(1,2),I(1,3),I(2,2),I(2,3),I(3,3)];
                
                robot.L(num,:) = [L(1,1),L(1,2),L(1,3),L(2,2),L(2,3),L(3,3)];
            end
            
            stdDynParams = [];
            baryParams = [];
            for num=1:robot.n
                baryParams = [baryParams, robot.I(num,:), robot.r(num,:), robot.m(num,:)];
                stdDynParams = [stdDynParams, robot.L(num,:), robot.l(num,:), robot.m(num,:)];
            end
            
            robot.stdDynParams = stdDynParams;
            robot.baryParams = baryParams;
            
            robot.isSymbolic = false;
        end
            
        function initSymbolicParams(robot)
            
            robot.m = sym('m',[robot.n,1]);
            robot.l = sym(zeros(robot.n,3));
            robot.r = sym(zeros(robot.n,3));
            
            robot.I = sym(zeros(robot.n,6));
            robot.L = sym(zeros(robot.n,6));
            

            for num=1:robot.n
                ln = [sym(strcat('l',num2str(num),'x'));
                    sym(strcat('l',num2str(num),'y'));
                    sym(strcat('l',num2str(num),'z'))];
                rn = [sym(strcat('r',num2str(num),'x'));
                    sym(strcat('r',num2str(num),'y'));
                    sym(strcat('r',num2str(num),'z'))];
                IVec = [sym(strcat('I',num2str(num),'xx'));
                    sym(strcat('I',num2str(num),'xy'));
                    sym(strcat('I',num2str(num),'xz'));
                    sym(strcat('I',num2str(num),'yy'));
                    sym(strcat('I',num2str(num),'yz'));
                    sym(strcat('I',num2str(num),'zz'))];
                LVec = [sym(strcat('L',num2str(num),'xx'));
                    sym(strcat('L',num2str(num),'xy'));
                    sym(strcat('L',num2str(num),'xz'));
                    sym(strcat('L',num2str(num),'yy'));
                    sym(strcat('L',num2str(num),'yz'));
                    sym(strcat('L',num2str(num),'zz'))];
                robot.I(num,:) = IVec;
                robot.L(num,:) = LVec;
                robot.l(num,:) = ln;
                robot.r(num,:) = rn;
            end
            
            stdDynParams = [];
            baryParams = [];
            for num=1:robot.n
                baryParams = [baryParams, robot.I(num,:), robot.r(num,:), robot.m(num,:)];
                stdDynParams = [stdDynParams, robot.L(num,:), robot.l(num,:), robot.m(num,:)];
            end
            
            robot.stdDynParams = stdDynParams;
            robot.baryParams = baryParams;
            robot.isSymbolic = true;
        end
        
        function setGravity(robot, gravity)
            %setGravity set the gravity of the robot.
            %   Default: [0,0,-g] (g is sym)
            robot.gravity = gravity;
        end
        
        function [posCOM, velCOM, w] = calculateGeometry(robot)
            syms t
            posCOM = sym(zeros(robot.n, 3));
            velCOM = sym(zeros(robot.n, 3));
            w = sym(zeros(robot.n, 3));
            
            for num=1:robot.n
                T0n = simplify(robot.rtbRobot.A(1:num, robot.q_t).T);
                posCOM_Homogen = T0n*[rByML(robot.m(num,:), robot.l(num,:)),1].';
                posCOM(num,:) = simplify(posCOM_Homogen(1:3).');
                velCOM(num,:) = simplify(diff(posCOM(num,:), t));
                velCOM(num,:) = subs(velCOM(num,:), [robot.diff_q], [robot.dq_t]);
                dRdt = diff(T0n(1:3,1:3), t);
                dRdt = subs(dRdt, [robot.diff_q], [robot.dq_t]);
                wn = T0n(1:3,1:3).'*dRdt;
                w(num,:) = simplify([wn(3,2), wn(1,3), wn(2,1)]);
            end
        end
        
        function [k,u] = calculateEnergy(robot)
            [posCOM, velCOM, w] = robot.calculateGeometry();
            k = robot.kineticEnergy(velCOM, w);
            u = robot.potentialEnergy(posCOM);
            robot.k = k;
            robot.u = u;
        end
        
        function k = kineticEnergy(robot, velCOM, w)
            k = sym(0);
            for num=1:robot.n
                disp(strcat("calculating kinetic energy for link ", num2str(num)))
                IbyLlm = IByLLM(robot.L(num,:), robot.m(num,:), rByML(robot.m(num,:), robot.l(num,:)));
                kn = (robot.m(num,:)*(velCOM(num,:)*velCOM(num,:).'))/2 + ...
                    (w(num,:) * IbyLlm * w(num,:).')/2;
                disp("Simplifying")
                if robot.isSymbolic
                    kn = prod(factor( expand(kn) - subs(expand(kn * robot.m(num,:)), robot.m(num,:), 0)/robot.m(num,:) ) );
                else
                    kn = simplify(kn);
                end
                k = k + kn;
            end
        end
        
        function u = potentialEnergy(robot,posCOM)
            u = sym(0);
            for num=1:robot.n
                disp(strcat("calculating potential energy for link ", num2str(num)))
                un = - robot.m(num,:)*(posCOM(num,:)*robot.gravity.');
                disp("Simplifying")
                un = expand(un);
                u = u + un;
            end

        end
        
        function [tau] = calculateJointTorque(robot)
            syms t;
            
            tau = sym(zeros(robot.n,1));
            
            robot.G = sym(zeros(robot.n,1));
            
            
            for num=1:robot.n
                disp(strcat("calculating joint torque for link ", num2str(num)))
                dkddottheta_n = diff(robot.k, robot.dq_t(num));
                dkdtheta_n = diff(robot.k, robot.q_t(num));
                dudtheta_n = diff(robot.u, robot.q_t(num));
                dkdt = diff(dkddottheta_n, t);
                dkdt = subs(dkdt, [robot.diff_q, robot.diff_dq],[robot.dq_t, robot.ddq_t]);
                tau(num,:) = dkdt - dkdtheta_n + dudtheta_n;
                % Gravity term
                disp(strcat("calculating gravity term for link ", num2str(num)))
                robot.G(num,:) = subs(dudtheta_n, [robot.q_t, robot.dq_t, robot.ddq_t], [robot.q, robot.dq, robot.ddq]);
            end
            tau = subs(tau, [robot.q_t, robot.dq_t, robot.ddq_t], [robot.q, robot.dq, robot.ddq]);
            robot.tau = tau;
            
        end
        
        function [M, C, G] = calculateModelMatrices(robot)
            M = sym(zeros(robot.n,robot.n));
%             C = sym(zeros(robot.n,1));
            G = robot.G;
            disp("calculating M matrix");
            for num=1:robot.n
                disp(strcat("calculating M matrix line ", num2str(num)));
                M(num,:) = equationsToMatrix(robot.tau(num,:), robot.ddq);
            end
            disp("calculating C matrix");
            C = subs(robot.tau, robot.ddq, zeros(1,robot.n)) - G;
            
            robot.M = M;
            robot.C = C;
        end
        
        function Y = calculateRegressor(robot)
            if robot.isSymbolic
                Y = sym(zeros(robot.n,10*robot.n));
                for num=1:robot.n
                    disp(strcat("calculating Y for joint ", num2str(num)))
                    Y(num,:) = equationsToMatrix(robot.tau(num,:), robot.stdDynParams);
                end
                Y = subs(Y, [robot.q_t,robot.dq_t,robot.ddq_t], [robot.q,robot.dq,robot.ddq]);

                robot.Y = Y;
            else
                % BE CAUTIOUS! This part is experimental. Errors may occur.
                Y = sym(zeros(robot.n,10*robot.n));
                
                IVecInL_sym = sym(zeros(robot.n,6));
                LVec_sym = sym(zeros(robot.n,6));
                l_sym = sym(zeros(robot.n,3));
                rInl_sym = sym(zeros(robot.n,3));
                for num=1:robot.n
                    ln = [sym(strcat('l',num2str(num),'x'));
                        sym(strcat('l',num2str(num),'y'));
                        sym(strcat('l',num2str(num),'z'))];
                    
                    LVec = [sym(strcat('L',num2str(num),'xx'));
                        sym(strcat('L',num2str(num),'xy'));
                        sym(strcat('L',num2str(num),'xz'));
                        sym(strcat('L',num2str(num),'yy'));
                        sym(strcat('L',num2str(num),'yz'));
                        sym(strcat('L',num2str(num),'zz'))];
                    LVec_sym(num,:) = LVec;
                    l_sym(num,:) = ln;
                    rInl_sym(num,:) = l_sym(num,:)./robot.m(num,:);
                    ITensorInL =  L2I(IVec2Tensor(LVec_sym(num,:)),robot.m(num,:),rInl_sym(num,:));
                    IVecInL_sym(num,:) = [ITensorInL(1,1),ITensorInL(1,2),ITensorInL(1,3),ITensorInL(2,2),ITensorInL(2,3),ITensorInL(3,3)];
                end
                stdDynParams_sym = [];
                baryParamsInDynParams_sym = [];
                for num=1:robot.n
                    baryParamsInDynParams_sym = [baryParamsInDynParams_sym, IVecInL_sym(num,:), rInl_sym(num,:), robot.m(num,:)];
                    stdDynParams_sym = [stdDynParams_sym, LVec_sym(num,:), l_sym(num,:), robot.m(num,:)];
                end
                for num=1:robot.n
                    disp(strcat("calculating Y for joint ", num2str(num)))
                    class(robot.stdDynParams)
                    tau_num = simplify(robot.tau(num,:));
                    tau_num = subs(tau_num,robot.baryParams,baryParamsInDynParams_sym);
                    tau_num = simplify(tau_num);
                    Y(num,:) = equationsToMatrix(tau_num, stdDynParams_sym);
                    Y(num,:) = subs(Y(num,:), stdDynParams_sym, robot.stdDynParams);
                end
                Y = subs(Y, [robot.q_t,robot.dq_t,robot.ddq_t], [robot.q,robot.dq,robot.ddq]);

                robot.Y = Y;
            end
        end
        
        function [Yb, baseParams] = findIndependentRegressor(robot)
            
            syms g
            % Find dependency
            paramNum = length(robot.stdDynParams);
            sampleNum = paramNum*2;
            Z = zeros(robot.n*sampleNum, paramNum);
            reverseStr = '';
            disp("Random sampling")
            for i=1:sampleNum
                % disp(strcat("random sampling ", num2str(i)))
                q_rand = rand(1,robot.n)*2*pi - pi;
                dq_rand = rand(1,robot.n)*2*pi - pi;
                ddq_rand = rand(1,robot.n)*2*pi - pi;
                Z((i-1)*robot.n+1:i*robot.n, :) = double(subs(robot.Y, [robot.q,robot.dq,robot.ddq,g], [q_rand,dq_rand,ddq_rand,9.81]));
                sampleDone = i;
                msg = sprintf('Progress: %3.0f/%3.0f', sampleDone, sampleNum);
                fprintf([reverseStr, msg]);
                reverseStr = repmat(sprintf('\b'), 1, length(msg));
            end
            fprintf('\n')
            disp('Computing...')
            rk = rank(Z); % rank
            [~,R,P] = qr(Z);
            R1 = R(1:rk, 1:rk);
            R2 = R(1:rk, rk+1:size(R,2));
            P_X = [eye(rk), R1\R2] * P.';
            
            P_X = round(P_X, 8);
            
            baseParams = P_X * robot.stdDynParams.';
            [rowNum,~] = find(P==1);
            Yb = robot.Y(:, rowNum(1:rk));
            robot.Yb = Yb;
            robot.baseParams = baseParams;
            
            
        end
        

    end
end

