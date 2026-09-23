ft_size=25;

L1error=(sum(abs(L2OUT(1:Tf/delta_t+1)-L2sp))+sum(abs(L1OUT(1:Tf/delta_t+1)-8.4375)))/size(L2OUT,1);
MAE1=L1error


L2error=(sum(abs(L2OUT1(1:Tf/delta_t+1)-L2sp2))+sum(abs(L1OUT1(1:Tf/delta_t+1)-6.75)))/size(L2OUT1,1);
MAE2=L2error

L3error=(sum(abs(L2OUT2(1:Tf/delta_t+1)-L2sp))+sum(abs(L1OUT2(1:Tf/delta_t+1)-8.4375)))/size(L2OUT2,1);
MAE3=L3error


L4error=(sum(abs(L2OUT3(1:Tf/delta_t+1)-L2sp2))+sum(abs(L1OUT3(1:Tf/delta_t+1)-6.75)))/size(L2OUT3,1);
MAE4=L4error

MAE = MAE1+MAE2+MAE3+MAE4
Cost1 = [Cost1;MAE1];
Cost2 = [Cost2;MAE2];
Cost3 = [Cost3;MAE3];
Cost4 = [Cost4;MAE4];