function[trace0,trace1,trace2,trace3,trace4]=TraceGeneration(num,ratio)
trace0=ones(1,num);
c=randperm(num);
trace1=trace0;
trace2=trace0;
trace3=trace0;
trace4=trace0;
loss=ratio*num;

for i=1:loss
    trace1(c(i))=0;
end

d=randperm(round(num/2));

for i=1:round(loss/2)
    trace2(d(i)*2)=0;
    trace2(d(i)*2-1)=0;
end

e=randperm(round(num/3));

for i=1:round(loss/3)
    for j=0:2
        trace3(e(i)*3-j)=0;
    end
end

f=randperm(round(num/4));

for i=1:round(loss/4)
    for j=0:3
        trace4(f(i)*4-j)=0;
    end
end
