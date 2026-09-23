    clear;
    clc;
    load samples.mat;
%     load solutionb.mat;
    solution = [];
for i = 1:960
    data.n  = 3;                     % number of states
    data.L  = 5;                     % superframe length
    data.N  = 4;                     % number of control loops
    data.P  = eye(data.N*data.n);    % solution to MSS Lyapunov equation
    data.ub = data.L - 1;            % upper bound on transmission exponent, Tx
    
    % Constraint matrices
    a1          = ones(1, data.L);
    a2          = 0:data.ub;
    
    opt.beq     = ones(data.N,1);    % RHS of equality constraints
    opt.bineq   = data.ub;           % RHS of inequality constraints
    opt.Aeq     = []; 
    opt.Aineq   = [];
    opt.S       = [];                % Matrix to retrieve Tx from t_tilde
    for j = 1:data.N
        opt.Aeq     = blkdiag(opt.Aeq, a1); % LHS of equality constraints
        opt.Aineq   = [opt.Aineq, a2];      % LHS of ineq constraints
        opt.S       = blkdiag(opt.S, a2);   
    end
    
    % Bounds on t-tilde search space
    opt.lb = 0*ones(1, data.L*data.N).';    
    opt.ub = 1*ones(1, data.L*data.N).';
    
    % Convexification parameter
    opt.delta = 1e-8;
    zo = [z1os(i,:)' z2os(i,:)' z3os(i,:)' z4os(i,:)'];
    zc = [z1cs(i,:)' z2cs(i,:)' z3cs(i,:)' z4cs(i,:)'];
    beta = 1- prrs(i,:);
    Q = [];
        for j = 1:data.N
            q = zeros(1, data.L);
            for jj = 0:data.ub
                q(:,jj+1) = zc(:,j)'*zc(:,j) + (zo(:,j)'*zo(:,j) - zc(:,j)'*zc(:,j))*beta(j)^(jj);
            end
            Q=[Q,q];
        end
        
        % Solving relaxed QP
        options = optimoptions('linprog', 'Display', 'off');
        soln_qp_rlx = linprog(Q, opt.Aineq, opt.bineq, opt.Aeq, opt.beq, opt.lb, opt.ub, [], options);
    
        % Converting back to Tx
        sol = round(opt.S*soln_qp_rlx);
        Tx1=sol(1);
        Tx2=sol(2);
        Tx3=sol(3);
        Tx4=sol(4);
        solution = [solution; Tx1 Tx2 Tx3 Tx4];
end
%     error = solution-solutionb;