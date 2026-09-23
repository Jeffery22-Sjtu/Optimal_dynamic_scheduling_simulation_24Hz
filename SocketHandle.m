%%Matlab function for handling socket communications between Matlab and
% Python event server (on Docker container)

function result=SocketHandle(uu1,uu2,uu3,uu4,slot1,slot2,slot3,slot4)
    t = tcpip('172.16.11.199', 10000);%my computer is 172.27.85.72
    fopen(t);
    %mode=3.14159;
%     if abs(ymea)<0.01
%         ymea=0;
%     end
    message =  sprintf('%08.3g%08.3g%08.3g%08.3g%04g%04g%04g%04g',uu1,uu2,uu3,uu4,slot1,slot2,slot3,slot4);
%     message =  sprintf('%08g',ymea);
%     message =  sprintf('%08g',-0.5);
    %message =strcat(num2str(ymea));
    result=message
    fwrite(t,message);    
    fclose(t);    
end