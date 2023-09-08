close all;

dt = time(2:end)-time(1:end-1);
figure; plot(dt,'LineWidth',0.85);grid on; title('Evolution of the sampling time'); xlabel('Sample'); ylabel('Sampling time [s]'); 
time=0:0.01:(length(pos)-1)*0.01;

%% ILLUSTRATE POSITION FILTERING
% yaw = yaw*2*pi/360;


filt_pos = test_filter(pos-pos(1),70,5)+pos(1);
err_RMS_pos = sqrt(sum((filt_pos-pos(1:length(filt_pos))).^2)/length(filt_pos))



%% Velocity filtering


vel = (filt_pos(2:end)-filt_pos(1:end-1))./(time(2:length(filt_pos))-time(1:length(filt_pos)-1));
filt_vel = test_filter(vel-vel(1),50,5)+vel(1); %50

err_RMS_vel = sqrt(sum((filt_vel-vel(1:length(filt_vel))).^2)/length(filt_vel))


% figure; plot(yaw_vel,'LineWidth',0.85);grid on; title('Computation of the yaw velocity with modified sampling time'); xlabel('Sample'); ylabel('Yaw velocity [rad/s]');hold on; plot(-amp *freq* sin(freq*time) - 0.5*amp*0.8*freq*sin(0.8*freq*time) + 1.2*amp*0.5*freq*sin(0.5*freq*time)-0.5*1.3*freq*amp*sin(1.3*freq*time)); legend('Computations based on the measurements','Commanded velocity')
%% Torque filtering

new_meas_eff = meas_eff - cog3(1:length(meas_eff));

%% Acceleration filtering



acc = (filt_vel(2:end)-filt_vel(1:end-1))./(time(2:length(filt_vel))-time(1:length(filt_vel)-1));
filt_acc = test_filter(acc-acc(1),70,5)+acc(1);

rms_err_acc = sqrt(sum((filt_acc-acc(1:length(filt_acc))).^2)/length(filt_acc))

%% Preparation
Y_act = [filt_acc(1:end)'];
fric_est = zeros(1,length(filt_acc));
counter = 1

%% stuff
res_act=lsqr(Y_act,(new_meas_eff(1:length(filt_acc))-fric_est(1:length(filt_acc)))')
J_act=res_act(1);

%,filt_vel(1:length(filt_acc))',(sign(filt_vel(1:length(filt_acc))))',(filt_pos(1:length(filt_acc)))'
model_act = 1*J_act*filt_acc + cog3(1:length(filt_acc));
y = meas_eff(1:length(model_act)) - model_act;
x = filt_vel(1:length(model_act)); 
mdl = @(beta,x) ((beta(1)+beta(2)*exp(-(x/beta(3)).^2) ).*sign(x) + beta(4)*x);
x_cell = {x};
y_cell = {y};
mdl_cell = {mdl};
beta0 = [1, 1, 1, 1];
[ress,r,J,Sigma,mse,errorparam,robustw] = nlinmultifit(x_cell, y_cell, mdl_cell, beta0);
fric_est = (ress(1)+ress(2)*exp(-(filt_vel/ress(3)).^2) ).*sign(filt_vel) + ress(4)*filt_vel;
new_mod_test(counter) = rms(meas_eff(1:length(model_act))-(model_act+fric_est(1:length(model_act))))
counter = counter+1



