clear all
close all
ADPS_bit = 3;
% ADPS_bit = 1; ADPS
% ADPS_bit = 2; Brute force
% ADPS_bit = 3; Linear laxation
% ADPS_bit = 4; Hueristic
NN=20;
Run_count = 500;
% 生成随机权重矩阵
% 存储1000次统计结果
exe_t_1000 = zeros(Run_count,NN);
for stime = 1:NN
    FL= stime;
    N= stime;
    w=abs(rand(N,1));
    for  ci = 1:Run_count
        t = 0;
        ONL=500*rand(N,1);
        CNL=500*rand(N,1);
        xoN1L = 30*rand(N,1);
        xoN2L = 30*rand(N,1);
        u0NL  = 100*rand(N,1);
        xcN1L = 30*rand(N,1);
        xcN2L = 30*rand(N,1);
        uCNL  = 100*rand(N,1);
        LpC2_N=30*rand(3,N);
        LpO2_N=30*rand(3,N);
        uC2_N=100*rand(1,N);
        zNo = zeros(N,3);
        zNc = zeros(N,3);
        for i=1:N
            zNo(i,1)=w(i)*xoN1L(i);
            zNo(i,2)=0.1*w(i)*xoN2L(i);
            zNo(i,3)=0.0001*w(i)*u0NL(i);
        end
        for i=1:N
            zNc(i,1)=w(i)*xcN1L(i);
            zNc(i,2)=0.1*w(i)*xcN2L(i);
            zNc(i,3)=0.0001*w(i)*uCNL(i);
        end
        delta_tc=abs(randn(N,1));
        prrN =abs(randn(N,1));
        TTN =abs(randn(N,1));
        data.n  = 3;                     % number of states
        data.L  = 5;                     % superframe length
        data.N  = N;                     % number of control loops
        zoN= zNo';                       % zNo 是 N*3 （行状态向量）的组合
        zcN= zNc';                       % zNc 是 N*3 （行状态向量）的组合
        prrN;
        beta1=1-prrN;
        prr2N=prrN+TTN;
        beta2=1-prr2N;
        loss1=zeros(N,1);
        gain1=zeros(N,1);
        for i=1:N
            if beta2(i)>1
                beta2(i)=1;
            end
            if beta2(i)<0
                beta2(i)=0;
            end
        end
        tsolution = [inf, zeros(1,N)];
        solution = [inf, zeros(1,N)];
        solution_set = [];
        tic;
        for j = 1:data.N
            loss1(j)=abs(zoN(:,j)'*zoN(:,j) - zcN(:,j)'*zcN(:,j));
            gain1(j)=zcN(:,j)'*zcN(:,j);
        end
        sol=zeros(1,data.N);
        if ADPS_bit == 1 % ADPS one-step prediction. optimal solution
            for jj=1:data.L-1
                delta=loss1.*(1-beta1);
                [Y,I]=sort(delta);
                loss1(I(data.N))=loss1(I(data.N))-loss1(I(data.N)).*(1-beta1(I(data.N))^1);
                sol(I(data.N))=sol(I(data.N))+1;
            end
            exe_t_1000(ci,stime) = toc;
        elseif ADPS_bit == 2
            solution = [inf, zeros(1,N)];
            TX=zeros(1,N);
            TX1=zeros(1,N);
            i=0;
            while sum(TX1)~=N
                i=i+1;
                for ik=1:NN % 最大的控制回路限制为30个回路
                    if i< 2^(ik)
                        break;
                    end
                end
                x = cell2mat(mat2cell(dec2bin(i), 1, ones(ik, 1)));
                for iu=1:ik
                    TX1(1,N-iu+1)=str2num(x(ik-iu+1));
                end
                if  sum(TX1)<=FL
                    Z=[];
                    for qq=1:N
                        Z=[Z; (1-prrN(qq))^TX1(N-qq+1)*zNo(qq,:)'+(1-((1-prrN(qq))^TX1(N-qq+1)))*zNc(qq,:)'];
                    end
                    Cost = Z'*Z;
                    Cost = 0;
                    for qq=1:N
                        ktemp= (1-prrN(qq))^TX1(N-qq+1)*zNo(qq,:)*zNo(qq,:)'+(1-(1-prrN(qq))^TX1(N-qq+1))*zNc(qq,:)*zNc(qq,:)';
                        Cost=Cost+ktemp;
                    end
                    if Cost<solution(1,1) && sum(TX1)<=FL
                        solution = [Cost, TX1];
                    end
                end
            end
            exe_t_1000(ci,stime) = toc;
        elseif ADPS_bit == 3
            tic;
            data.n  = 3;                     % number of states
            data.L  = FL+1;                     % superframe length
            data.N  = N;                     % number of control loops
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
            Q = [];
            for j = 1:data.N
                q = zeros(1, data.L);
                for jj = 0:data.ub
                    q(:,jj+1) =zNc(j,:)*zNc(j,:)' + (zNo(j,:)*zNo(j,:)' - zNc(j,:)*zNc(j,:)')*beta1(j)^(jj);
                end
                Q=[Q,q];
            end
            % Solving relaxed QP
            options = optimoptions('linprog', 'Display', 'off');
            soln_qp_rlx = linprog(Q, opt.Aineq, opt.bineq, opt.Aeq, opt.beq, opt.lb, opt.ub, [], options);
            exe_t_1000(ci,stime) = toc;

        elseif ADPS_bit == 4
            tic;
            for j = 1:data.N
                gain1(j)=zcN(:,j)'*zcN(:,j);
            end
            [Y1,I1]=sort(loss1);
            for iur=1:stime
                sol(I1(iur))=stime;
            end
            exe_t_1000(ci,stime) = toc;
        end
    end
end
% 存平均值的数组
mean_score=zeros(1,NN);
for i=1:NN
    [Y,I]=sort(exe_t_1000(:,i),'descend');
    exe_t_1000(:,i)=Y;
    mean_score(i)=mean(exe_t_1000(100:end,i));
end