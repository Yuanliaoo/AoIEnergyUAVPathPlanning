function [Trajectory,Avg_AoI,Energy] = Energy_solver()

load('parameters.mat');
[num_BS_user,~] = size(BS_user_locations);

%%
%cvx minimize the energy

cvx_solver gurobi_2

cvx_begin quiet
    variable Xij(num_BS_user,num_BS_user) binary
    variable Fij(num_BS_user,num_BS_user) nonnegative
    
    minimize sum(sum(Xij.*energy_matrix))
    
    subject to 
    
        sum(Xij(1,2:end)) == sum(Xij(2:end,1));
        
        for i = 2:num_BS_user
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
Trajectory = round(Xij);
Energy = cvx_optval;

%%
% There are might be two directions of TSP solution
% choose the one with the best AoI

time_did_num = time_matrix./(num_BS_user-1);
Avg_AoI_1 = sum(sum(round(Fij).*time_did_num));

Fij_inv = zeros(num_BS_user,num_BS_user);
for i = 1:num_BS_user
    for j = 1:num_BS_user
        if Trajectory(i,j) == 1
            Fij_inv(j,i) = (num_BS_user-1) - round(Fij(i,j));
        end
    end
end
Avg_AoI_2 = sum(sum(Fij_inv.*time_did_num));

if Avg_AoI_2 < Avg_AoI_1
    Avg_AoI = Avg_AoI_2;
    Trajectory = Trajectory';
else
    Avg_AoI = Avg_AoI_1;
end
    
end
