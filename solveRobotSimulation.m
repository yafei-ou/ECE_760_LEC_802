function dy = solveRobotSimulation(t,states,Mfun,Cfun,Gfun,tau)

    q = states(1:6);
    dq = states(7:12);

    g_num = -9.81;

    in1 = q;
    in2 = [q;dq];
    in3 = [q;g_num];
%     in1 = mat2cell(in1,ones(numel(in1),1),1);
%     in2 = mat2cell(in2,ones(numel(in2),1),1);
%     in3 = mat2cell(in3,ones(numel(in3),1),1);
% 
% 
%     M = Mfun(in1{:});
%     C = Cfun(in2{:});
%     G = Gfun(in3{:});
    M = M_fun(in1(1),in1(2),in1(3),in1(4),in1(5),in1(6));
    C = C_fun(in2(1),in2(2),in2(3),in2(4),in2(5),in2(6),in2(7),in2(8),in2(9),in2(10),in2(11),in2(12));
    G = G_fun(in3(1),in3(2),in3(3),in3(4),in3(5),in3(6),in3(7));

    ddq = M\(tau - C - G);

    dy = [dq;ddq];
    
end