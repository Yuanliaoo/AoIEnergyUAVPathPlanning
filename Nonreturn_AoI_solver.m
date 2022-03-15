function [Trajectory,Avg_AoI,Energy] = Nonreturn_AoI_solver()

load('parameters.mat');
[num_BS_user,~] = size(BS_user_locations);

%%
%cvx solve the average AoI

time_did_num = time_matrix./(num_BS_user-1);

cvx_solver gurobi_2

cvx_begin quiet
    variable Xij(num_BS_user,num_BS_user) binary
    variable Fij(num_BS_user,num_BS_user) nonnegative
    
    minimize sum(sum(Fij.*time_did_num))
    
    subject to 
        
        for i = 1:num_BS_user
            sum(Xij(i,:)) == 1;
            sum(Xij(:,i)) == 1; 
        end
        
        for i = 2:num_BS_user
            sum(Fij(i,:)) - sum(Fij(:,i)) == 1;
        end
        
        Fij(1,:) == 0;
        
        Fij <= num_BS_user.*Xij;
       
cvx_end

%%
Trajectory = Xij;
Avg_AoI = cvx_optval;
Energy = sum(sum(Trajectory.*energy_matrix));

end

