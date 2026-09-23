function [scheduled_slot] = draft_schedule(deadline,delta_tr,delta_t,slot_length,loop_num,priority,l1,l3)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% draft_schedule
% Draft schedule, first come first serve. If task comes at the same time,
% then the scheduling follow the order of priorities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


   global schedule
   period = floor((deadline+delta_tr/slot_length)/5);
   slot = deadline-(period-1)*5;
   if slot ==1||slot==0
       if priority == 1      
           slot = 2;
       elseif priority == 2
           if l1==0&&l3==0
               slot = 2;
           else
               slot = 3;
           end
       elseif priority ==3
           if l1==0&&l3==0
               slot = 2;
           else
               slot = 4;
           end
       end
   end
   
   if schedule(period,slot)==0
       scheduled_slot=(period*5+slot)*slot_length/delta_t;
   else
       assign = 0;
       for i =period:size(schedule,1)
           if i==period
               for j = slot+1: size(schedule,2)
                   if schedule(i,j)==0
                       assign=1;
                   end
                   if assign==1
                       break;
                   end
               end
           else
               for j = 2: size(schedule,2)
                   if schedule(i,j)==0
                       assign=1;
                   end
                   if assign==1
                       break;
                   end
               end
           end
           if assign==1
               break;
           end
       end
       scheduled_slot=(i*5+j)*slot_length/delta_t;
   end
end

