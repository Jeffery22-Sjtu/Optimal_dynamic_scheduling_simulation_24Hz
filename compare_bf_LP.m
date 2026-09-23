load('solution_set_error.mat')
fprintf('\nTotal number of errors: %d\n', norm(solution_set,1));
error = 0;
error_count = 0;
for i = 1:4
    for j = 1: 14400
        if solution_set(j,i)~=0
            i
            j
            error =error+abs(solution_set(j,i));
            error_count = error_count+1;
        end
    end
end
error
error_count