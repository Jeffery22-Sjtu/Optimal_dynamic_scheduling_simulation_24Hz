function[r0,r1,r2,r3,r4]=tracetest(num,trace0,trace1,trace2,trace3,trace4)
rr0=0;
rr1=0;
rr2=0;
rr3=0;
rr4=0;
for i=1:num
    if trace0(i)==0
        rr0=rr0+1;
    end
    if trace1(i)==0
        rr1=rr1+1;
    end
    if trace2(i)==0
        rr2=rr2+1;
    end
    if trace3(i)==0
        rr3=rr3+1;
    end
    if trace4(i)==0
        rr4=rr4+1;
    end
end
r0=rr0/num;
r1=rr1/num;
r2=rr2/num;
r3=rr3/num;
r4=rr4/num;
        