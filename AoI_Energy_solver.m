function [Trajectory,Avg_AoI,Energy] = AoI_Energy_solver(weights,AoI_bound,Energy_bound)

load('parameters.mat');
[num_BS_user,~] = size(BS_user_locations);

%%
%pre_calculate parameters
AoI_para = weights.AoI_weight/(AoI_bound.AoI_upper - AoI_bound.AoI_lower); 
Energy_para = weights.Energy_weight/(Energy_bound.Energy_upper - Energy_bound.Energy_lower);

%%
%cvx solve the energy+energy

time_did_num = time_matrix./(num_BS_user-1);

cvx_solver gurobi_2

cvx_begin quiet
    variable Xij(num_BS_user,num_BS_user) binary
    variable Fij(num_BS_user,num_BS_user) nonnegative
    
    minimize AoI_para*(sum(sum(Fij.*time_did_num)) - AoI_bound.AoI_lower)...
        + Energy_para*(sum(sum(Xij.*energy_matrix)) - Energy_bound.Energy_lower)
        
    
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
Trajectory = Xij;
Avg_AoI = sum(sum(Fij.*time_did_num));
Energy = sum(sum(Trajectory.*energy_matrix));
%gplot(Trajectory,BS_user_locations,'-k');


end
