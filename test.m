%%
%Calculate the upper and lower bound of energy and AoI
[~,AoI_lower,Energy_upper] = AoI_solver();
[~,AoI_upper,Energy_lower] = Energy_solver();

AoI_bound = table(AoI_lower,AoI_upper);
Energy_bound = table(Energy_lower,Energy_upper);

%%
% Calculate the proposed multi-return-allowed mode
AoI_weight = 0.5; Energy_weight = 0.5; weights = table(AoI_weight,Energy_weight);
[Trajectory_05_05,Avg_AoI_05_05,Energy_05_05] = AoI_Energy_solver(weights,AoI_bound,Energy_bound);

%%
% Calculate the path from reference [1]
[Trajectory_nonreturn,Avg_AoI_nonreturn,Energy_nonreturn] = Nonreturn_AoI_solver();