close all;
clear;
D = 'C:\Users\maxjo\Downloads\UNLOADED_YAW_COG';
S = dir(fullfile(D,'*.mat'));
N = numel(S);
%C = cell(29,N);

N_DATA = 16000;

% First we make sure everything has the same size to concatenate everything

for k = 1:12

    F = fullfile(D,S(k).name);
    C2 = struct2cell(load(F,'pos','meas_eff'));
    oldN = cellfun(@numel, C2); % old lengths of cell elements
    old_length(:,k) = oldN;
    newN = N_DATA*ones(2,1);
    padfun = @(g) [C2{g} zeros(1, newN(g) - oldN(g))] ;
    C3 = arrayfun(padfun, 1:numel(C2) , 'un', 0) ; % apply padding to all elements of C
    C(:,k) = reshape(C3,size(C2));

end

pos = vertcat(C{1,:});
meas_eff = vertcat(C{2,:});
time_vec = 0:0.01:150;
for j = 1:4
    plot(time_vec(1:500),meas_eff(j,1:500),'LineWidth',1.25); hold on; grid on;
end



% pos = pos(:,1:14990);
% meas_eff = meas_eff(:,1:14990);
time = ones(16,1)*time_vec;


%% POSITION FILTERING


for h=1:k
    lfp(h)=length(test_filter(pos(h,1:old_length(1,h)),70,5));
    filt_pos(h,1:16000) = [test_filter(pos(h,1:old_length(1,h))-pos(h,1),70,5)+pos(h,1),zeros(1,16000-lfp(h))];
    
end




%% Velocity filtering

lfp=size(filt_pos)
%We always go up to 16000 to be able to concatenate everything, we remove
%all the useless zeros after

for h=1:k

    vel(h,1:16000) = [0,(filt_pos(h,2:lfp(h))-filt_pos(h,1:lfp(h)-1))./(time(h,2:lfp(h))-time(h,1:lfp(h)-1)),zeros(1,16000-lfp(h))];
    lfv(h)=length(test_filter(vel(h,1:lfp(h)),70,5));
    filt_vel(h,1:16000) = [test_filter(vel(h,1:lfp(h))-vel(h,1),70,5)+vel(h,1),zeros(1,16000-lfv(h))];

end



%% IDENTIFICATION


stribeck_term = exp(-80*filt_vel(1:length(filt_acc)).^2).*sign(filt_vel(1:length(filt_acc)));
Y_act = [filt_acc(1:end)',filt_vel(1:length(filt_acc))',(sign(filt_vel(1:length(filt_acc))))',stribeck_term'];
res_act=lsqr(Y_act,(meas_eff(1:length(filt_acc)))')
J_act=res_act(1); C_act=res_act(2); Fric_act = res_act(3); Fric_str_act = res_act(4);



model_act = J_act*filt_acc+C_act*filt_vel(1:length(filt_acc))+Fric_act*sign(filt_vel(1:length(filt_acc)))+ Fric_str_act*stribeck_term+cog3(1:length(filt_acc));

final_rms_act = sqrt(sum((model_act-meas_eff(1:length(model_act))).^2)/length(model_act))

figure; plot(time,meas_eff); hold on; grid on; plot(time(1:length(model_act)),model_act); legend('Measured torque', 'Model based on actuator measurements');title('Model based on actuator measurements')

% filt_current = test_filter(windingCurrent,30,5);
% figure;
% plot(meas_eff); hold on; grid on; plot(meas_eff(1:length(filt_vel))+3*filt_vel); plot(8.8*filt_current); legend('Torque','Torque + vel', 'Current')
% 
% 
% % plot(q0-amp+amp * cos(freq*time) + 0.5*amp*cos(0.8*freq*time) - 1.2*amp*cos(0.5*freq*time)+0.5*amp*cos(1.3*freq*time))
% % plot(-amp *freq* sin(freq*time) - 0.5*amp*0.8*freq*sin(0.8*freq*time) + 1.2*amp*0.5*freq*sin(0.5*freq*time)-0.5*1.3*freq*amp*sin(1.3*freq*time))
% % plot(-amp *freq* cos(freq*time) - 0.5*amp*0.8^2*freq^2*cos(0.8*freq*time) + 1.2*amp*0.5^2*freq^2*cos(0.5*freq*time)-0.5*1.3^2*freq^2*amp*cos(1.3*freq*time))
% 
% Y = [filt_acc(200:end)',filt_vel(200:length(filt_acc))',filt_pos(200:length(filt_acc))',ones(length(filt_acc)-199,1)];
% res=lsqr(Y,(filt_trq(200:length(filt_acc)))'); J=res(1); C=res(2); G = res(3); Fric = res(4);
% 
% 
% inertia_and_friction_sol = J*filt_acc+C*filt_vel(1:length(filt_acc));

% inertia_and_friction_gravity = J*filt_acc+C*filt_vel(1:length(filt_acc))+G*filt_pos(1:length(filt_acc))+Fric;
% figure;
% plot(filt_trq); hold on; grid on; plot(inertia_and_friction_sol); plot(inertia_and_friction_gravity); legend('Measured torque','No gravity','With gravity')
% err_no_gravity = sum((filt_trq(1:length(inertia_and_friction_sol))-inertia_and_friction_sol).^2)
% err_gravity = sum((filt_trq(1:length(inertia_and_friction_gravity))-inertia_and_friction_gravity).^2)
% 
% figure;
% plot(filt_trq(1:length(inertia_and_friction_sol))-inertia_and_friction_gravity); grid on; legend('Mismatch');

testtt = 0:1:1000;

for nb=1:length(testtt)
    stribeck_term = exp(-testtt(nb)*filt_vel(1:length(filt_acc)).^2).*sign(filt_vel(1:length(filt_acc)));
    Y_act = [filt_acc(1:end)',filt_vel(1:length(filt_acc))',(sign(filt_vel(1:length(filt_acc))))',stribeck_term'];
    res_act=lsqr(Y_act,(new_meas_eff(1:length(filt_acc)))');
    J_act=res_act(1); C_act=res_act(2); Fric_act = res_act(3); Fric_str_act = res_act(4);



    model_act = J_act*filt_acc+C_act*filt_vel(1:length(filt_acc))+Fric_act*sign(filt_vel(1:length(filt_acc)))+ Fric_str_act*stribeck_term+cog3(1:length(filt_acc));

    final_rms_act2(nb) = sqrt(sum((model_act-meas_eff(1:length(model_act))).^2)/length(model_act));

end
plot(final_rms_act2)