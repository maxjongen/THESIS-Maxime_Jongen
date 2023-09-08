close all;

dt = time(2:end)-time(1:end-1);
figure; plot(dt,'LineWidth',0.85);grid on; title('Evolution of the sampling time'); xlabel('Sample'); ylabel('Sampling time [s]'); 
time=0:0.01:(length(pos)-1)*0.01;

%% ILLUSTRATE POSITION FILTERING
% yaw = yaw*2*pi/360;

spl = 1:2;
% spl = 1:length(filt_yaw);
pos = pos-pos_ref;
minpos = min(pos)
maxpos = max(pos)
filt_pos = test_filter(pos-pos(1),70,5)+pos(1);
figure; plot(spl,pos(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_pos(spl),'LineWidth',0.85); legend('Actuator measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw angle [rad]'); title('Effect of filtering on actuator measurements');
err_RMS_pos = sqrt(sum((filt_pos-pos(1:length(filt_pos))).^2)/length(filt_pos))

figure; plot(time(1:length(filt_pos)),filt_pos,'LineWidth',0.85); grid on; hold on; plot(t1,q1);xlabel('Time [s]'); ylabel('Yaw angle [rad]'); title('Robotic trajectory tracking'); legend('Measured position', 'Position command')


%% Velocity filtering


vel = (filt_pos(2:end)-filt_pos(1:end-1))./(time(2:length(filt_pos))-time(1:length(filt_pos)-1));
filt_vel = test_filter(vel-vel(1),50,5)+vel(1); %50
figure; plot(spl,vel(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_vel(spl),'LineWidth',0.85); legend('Velocity computed with actuator measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw velocity [rad/s]'); title('Velocity computations from actuator measurements');

err_RMS_vel = sqrt(sum((filt_vel-vel(1:length(filt_vel))).^2)/length(filt_vel))


% figure; plot(yaw_vel,'LineWidth',0.85);grid on; title('Computation of the yaw velocity with modified sampling time'); xlabel('Sample'); ylabel('Yaw velocity [rad/s]');hold on; plot(-amp *freq* sin(freq*time) - 0.5*amp*0.8*freq*sin(0.8*freq*time) + 1.2*amp*0.5*freq*sin(0.5*freq*time)-0.5*1.3*freq*amp*sin(1.3*freq*time)); legend('Computations based on the measurements','Commanded velocity')
%% Torque filtering

new_meas_eff = meas_eff;% - cog3(1:length(meas_eff));
filt_trq = test_filter(meas_eff,25,2.5);

figure; plot(spl,meas_eff(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_trq(spl),'LineWidth',0.85); legend('Measured torque','Filtered signal'); xlabel('Sample'); ylabel('Torque [Nm]'); title('Filtering of the torque signal');
rms_err_trq = sqrt(sum((filt_trq-meas_eff(1:length(filt_trq))).^2)/length(filt_trq))
rmstrq = rms(meas_eff)

%% Acceleration filtering



acc = (filt_vel(2:end)-filt_vel(1:end-1))./(time(2:length(filt_vel))-time(1:length(filt_vel)-1));
filt_acc = test_filter(acc-acc(1),70,5)+acc(1);
figure; plot(spl,acc(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_acc(spl),'LineWidth',0.85); legend('Acceleration computed with actuator measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw acceleration [rad/s^2]'); title('Acceleration computations from actuator measurements');
rms_err_acc = sqrt(sum((filt_acc-acc(1:length(filt_acc))).^2)/length(filt_acc))

%% IDENTIFICATION
Y_act = [filt_acc(1:end)',filt_vel(1:length(filt_acc))',(sign(filt_vel(1:length(filt_acc))))'];
res_act=lsqr(Y_act,(meas_eff(1:length(filt_acc)))')
J_act=res_act(1); C_act=res_act(2); Fric_act = res_act(3);


model_act = 1*J_act*filt_acc+C_act*filt_vel(1:length(filt_acc))+Fric_act*sign(filt_vel(1:length(filt_acc)));%+cog3(1:length(filt_acc));

final_rms_act = sqrt(sum((model_act-meas_eff(1:length(model_act))).^2)/length(model_act))

figure; plot(time,meas_eff,'LineWidth',0.85); hold on; grid on; plot(time(1:length(model_act)),model_act,'LineWidth',0.85); legend('Measured torque', 'Model based on actuator measurements');title('Model from test 1: visualisation'); xlabel('Time [s]'); ylabel('Torque [Nm]')

rms(meas_eff)
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
