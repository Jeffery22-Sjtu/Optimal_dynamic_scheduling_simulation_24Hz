    clear;
    clc;
    load samples.mat;
    solutionb = [];
for i = 1:960
    sol = [100000 0 0 0 0];
    prr1 = prrs(i,1);
    prr2 = prrs(i,2);
    prr3 = prrs(i,3);
    prr4 = prrs(i,4);
    z1o = z1os(i,:);
    z2o = z2os(i,:);
    z3o = z3os(i,:);
    z4o = z4os(i,:);
    z1c = z1cs(i,:);
    z2c = z2cs(i,:);
    z3c = z3cs(i,:);
    z4c = z4cs(i,:);
    for Tx1=0:4
            for Tx2=0:4-Tx1
                for Tx3=0:4-Tx1-Tx2
                    for Tx4=0:4-Tx1-Tx2-Tx3
                        Z=[(1-prr1)^Tx1*z1o'+(1-(1-prr1)^Tx1)*z1c';(1-prr2)^Tx2*z2o'+(1-(1-prr2)^Tx2)*z2c';...
                            (1-prr3)^Tx3*z3o'+(1-(1-prr3)^Tx3)*z3c';(1-prr4)^Tx4*z4o'+(1-(1-prr4)^Tx4)*z4c'];    
                        Cost = (1-prr1)^Tx1*z1o*z1o'+(1-(1-prr1)^Tx1)*z1c*z1c'...
                            + (1-prr2)^Tx2*z2o*z2o'+(1-(1-prr2)^Tx2)*z2c*z2c'...
                            + (1-prr3)^Tx3*z3o*z3o'+(1-(1-prr3)^Tx3)*z3c*z3c'...
                            + (1-prr4)^Tx4*z4o*z4o'+(1-(1-prr4)^Tx4)*z4c*z4c';
                            if Cost<sol(1,1)
                                    sol = [Cost,Tx1,Tx2,Tx3,Tx4];
                            end
                    end
                end
            end       
    end

    Tx1=sol(2);
    Tx2=sol(3);
    Tx3=sol(4);
    Tx4=sol(5);
        solutionb = [solutionb; Tx1 Tx2 Tx3 Tx4];
    end