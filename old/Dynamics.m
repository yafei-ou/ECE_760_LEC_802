classdef Dynamics < handle
    %LINK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        n
        m
        r
        l % COM; m*r=l
        I
        L
%         rByml
%         IByLlm
        velocityLink
        velocityCOM
        wLink
        
        baryParams
        stdDynParams
        
        rtbRobot
        q
        dq
        ddq
        diff_q
        diff_dq
        qq
        dqq
        ddqq
        g
        k
        u
        tau
        Y
        Yb
        base_params
    end
    
    methods
        function robot = Dynamics(rtbRobot)
            %LINK Construct an instance of this class
            %   Detailed explanation goes here
            robot.n = rtbRobot.n;
            robot.rtbRobot = rtbRobot;
            
            robot.m = sym('m',[robot.n,1]);
            robot.l = sym(zeros(robot.n,3));
            robot.r = sym(zeros(robot.n,3));
            
            robot.I = sym(zeros(robot.n,6));
            robot.L = sym(zeros(robot.n,6));
            
%             robot.rByml = sym(zeros(robot.n,3));
%             robot.IByLlm = sym(zeros(robot.n,6));
            
            % assume 6 joints
            syms theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t)
            syms theta_dot_1(t) theta_dot_2(t) theta_dot_3(t) theta_dot_4(t) theta_dot_5(t) theta_dot_6(t)
            syms theta_ddot_1(t) theta_ddot_2(t) theta_ddot_3(t) theta_ddot_4(t) theta_ddot_5(t) theta_ddot_6(t)
            syms g
            
            robot.q = [theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t)];
            robot.dq = [theta_dot_1(t) theta_dot_2(t) theta_dot_3(t) theta_dot_4(t) theta_dot_5(t) theta_dot_6(t)];
            robot.ddq = [theta_ddot_1(t) theta_ddot_2(t) theta_ddot_3(t) theta_ddot_4(t) theta_ddot_5(t) theta_ddot_6(t)];

            robot.qq = sym("q", [1,robot.n]);
            robot.dqq = sym("dq", [1,robot.n]);
            robot.ddqq = sym("ddq", [1,robot.n]);
            
            robot.diff_q = diff(robot.q, t);
            robot.diff_dq = diff(robot.dq, t);
            
            robot.g = [0,0,-g];
            

            
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
        end
        
        function [posCOM, velCOM, w] = calculateGeometry(robot)
            syms t
            posCOM = sym(zeros(robot.n, 3));
            velCOM = sym(zeros(robot.n, 3));
            w = sym(zeros(robot.n, 3));
            
            for num=1:robot.n
                T0n = simplify(robot.rtbRobot.A(1:num, robot.q).T);
                posCOM_Homogen = T0n*[robot.rByML(robot.m(num,:), robot.l(num,:)),1].';
                posCOM(num,:) = simplify(posCOM_Homogen(1:3).');
                velCOM(num,:) = simplify(diff(posCOM(num,:), t));
                velCOM(num,:) = subs(velCOM(num,:), [robot.diff_q], [robot.dq]);
                dRdt = diff(T0n(1:3,1:3), t);
                dRdt = subs(dRdt, [robot.diff_q], [robot.dq]);
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
                IbyLlm = robot.IByLLM(robot.L(num,:), robot.m(num,:), robot.rByML(robot.m(num,:), robot.l(num,:)));
                kn = (robot.m(num,:)*(velCOM(num,:)*velCOM(num,:).'))/2 + ...
                    (w(num,:) * IbyLlm * w(num,:).')/2;
                disp("Simplifying")
%                 simplify(kn);
                kn = prod(factor( expand(kn) - subs(expand(kn * robot.m(num,:)), robot.m(num,:), 0)/robot.m(num,:) ) );
                k = k + kn;
            end
        end
        
        function u = potentialEnergy(robot,posCOM)
            u = sym(0);
            for num=1:robot.n
                disp(strcat("calculating potential energy for link ", num2str(num)))
                un = - robot.m(num,:)*(posCOM(num,:)*robot.g.');
                disp("Simplifying")
                un = expand(un);
                u = u + un;
            end

        end
        
        function tau = calculateJointTorque(robot)
            syms t;
            
            tau = sym(zeros(robot.n,1));
            for num=1:robot.n
                disp(strcat("calculating joint torque for link ", num2str(num)))
                dkddottheta_n = diff(robot.k, robot.dq(num));
                dkdtheta_n = diff(robot.k, robot.q(num));
                dudtheta_n = diff(robot.u, robot.q(num));
                dkdt = diff(dkddottheta_n, t);
                dkdt = subs(dkdt, [robot.diff_q, robot.diff_dq],[robot.dq, robot.ddq]);
                tau(num,:) = dkdt - dkdtheta_n + dudtheta_n;
                
            end
            robot.tau = tau;
        end
        
        function Y = calculateRegressor(robot)
            Y = sym(zeros(robot.n,10*robot.n));
            for num=1:robot.n
                disp(strcat("calculating Y for torque ", num2str(num)))
                Y(num,:) = equationsToMatrix(robot.tau(num,:), robot.stdDynParams);
            end
            Y = subs(Y, [robot.q, robot.dq, robot.ddq], [robot.qq, robot.dqq, robot.ddqq]);

            robot.Y = Y;
        end
        
        function Yb = findSmallRegressor(robot)
            Yb = 0;
        end
        
        function [M, C, G] = findModelMatrices(robot)
            M = equationsToMatrix(robot.tau, )
        end        
        
        function rbyml = rByML(~,m,l)
            rbyml = l./m;
        end
        
        function IbyLlm = IByLLM(robot, Lvec, m, rbyml)
            IbyLlm = robot.IVec2Tensor(Lvec) - m * robot.vec2so3(rbyml).' * robot.vec2so3(rbyml);
        end
        
        function mat = vec2so3(~,vec)
            mat = [0,-vec(3),vec(2);
                 vec(3),0,-vec(1);
                 -vec(2),vec(1),0];
        end
        
        function Itensor = IVec2Tensor(~,IVec)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            Itensor = [IVec(1), IVec(2), IVec(3);
                IVec(2), IVec(4), IVec(5);
                IVec(3), IVec(5), IVec(6)];
        end
    end
end

