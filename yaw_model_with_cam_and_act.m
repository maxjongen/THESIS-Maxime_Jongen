close all;

dt = time(2:end)-time(1:end-1);
figure; plot(dt,'LineWidth',0.85);grid on; title('Evolution of the sampling time'); xlabel('Sample'); ylabel('Sampling time [s]'); 
time=0:0.01:50;
%% PLOT TRAJECTORY

spl = 1:500;
figure; plot(time(spl),pos(spl)-pos(1),'LineWidth',1.25); grid on;  hold on;  plot(time(spl),yaw(spl)-yaw(1),'LineWidth',1.25);  xlabel('Time [s]'); ylabel('Yaw angle [rad]'); title('Uncorrelated position measurements'); legend('Actuator','Cameras')

%% ILLUSTRATE POSITION FILTERING
% yaw = yaw*2*pi/360;
spl = 1:length(pos)-300;
% spl = 1:length(filt_yaw);
yaw = yaw - yaw(1);
filt_yaw = test_filter(yaw-yaw(1),70,5)+yaw(1);
figure; plot(spl,yaw(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_yaw(spl),'LineWidth',0.85); legend('Camera measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw angle [rad]'); title('Effect of filtering on camera measurements');
err_RMS_yaw = sqrt(sum((filt_yaw-yaw(1:length(filt_yaw))).^2)/length(filt_yaw))


filt_pos = test_filter(pos-pos(1),70,5)+pos(1);
figure; plot(spl,pos(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_pos(spl),'LineWidth',0.85); legend('Actuator measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw angle [rad]'); title('Effect of filtering on actuator measurements');
err_RMS_pos = sqrt(sum((filt_pos-pos(1:length(filt_pos))).^2)/length(filt_pos))

figure; plot(time(1:length(filt_pos)),filt_pos,'LineWidth',0.85); grid on; xlabel('Time [s]'); ylabel('Yaw angle [rad]'); title('Position command');

filt_pos = filt_pos - filt_pos(1);
filt_yaw = filt_yaw - filt_yaw(1);
rms_2signals = sqrt(sum((filt_pos-filt_yaw(1:length(filt_pos))).^2)/length(filt_pos));
rms(yaw-pos)
%% Velocity filtering
spl = 6755:6880;
spl = 1:length(filt_yaw_vel);

yaw_vel = (filt_yaw(2:end)-filt_yaw(1:end-1))./(time(2:length(filt_yaw))-time(1:length(filt_yaw)-1)); 
filt_yaw_vel = test_filter(yaw_vel-yaw_vel(1),50,5)+yaw_vel(1);
figure; plot(spl,yaw_vel(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_yaw_vel(spl),'LineWidth',0.85); legend('Velocity computed with cameras measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw velocity [rad/s]'); title('Velocity computations from cameras measurements');
err_RMS_yaw_vel = sqrt(sum((filt_yaw_vel-yaw_vel(1:length(filt_yaw_vel))).^2)/length(filt_yaw_vel))

vel = (filt_pos(2:end)-filt_pos(1:end-1))./(time(2:length(filt_pos))-time(1:length(filt_pos)-1));
filt_vel = test_filter(vel-vel(1),50,5)+vel(1); %50
figure; plot(spl,vel(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_vel(spl),'LineWidth',0.85); legend('Velocity computed with actuator measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw velocity [rad/s]'); title('Velocity computations from actuator measurements');

err_RMS_vel = sqrt(sum((filt_vel-vel(1:length(filt_vel))).^2)/length(filt_vel))
rms_2signals_vel = sqrt(sum((filt_vel-filt_yaw_vel(1:length(filt_vel))).^2)/length(filt_vel))

% figure; plot(yaw_vel,'LineWidth',0.85);grid on; title('Computation of the yaw velocity with modified sampling time'); xlabel('Sample'); ylabel('Yaw velocity [rad/s]');hold on; plot(-amp *freq* sin(freq*time) - 0.5*amp*0.8*freq*sin(0.8*freq*time) + 1.2*amp*0.5*freq*sin(0.5*freq*time)-0.5*1.3*freq*amp*sin(1.3*freq*time)); legend('Computations based on the measurements','Commanded velocity')
%% Torque filtering

new_meas_eff = meas_eff; %- cog3(1:length(meas_eff));
filt_trq = test_filter(meas_eff,25,2.5);
spl = 1:length(filt_trq);
figure; plot(spl,meas_eff(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_trq(spl),'LineWidth',0.85); legend('Measured torque','Filtered signal'); xlabel('Sample'); ylabel('Torque [Nm]'); title('Filtering of the torque signal');
rms_err_trq = sqrt(sum((filt_trq-meas_eff(1:length(filt_trq))).^2)/length(filt_trq))


%% Acceleration filtering
spl = 2900:3080;
yaw_acc = (filt_yaw_vel(2:end)-filt_yaw_vel(1:end-1))./(time(2:length(filt_yaw_vel))-time(1:length(filt_yaw_vel)-1));
filt_yaw_acc = test_filter(yaw_acc-yaw_acc(1),70,5)+yaw_acc(1);
figure; plot(spl,yaw_acc(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_yaw_acc(spl),'LineWidth',0.85); legend('Acceleration computed with cameras measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw acceleration [rad/s^2]'); title('Acceleration computations from cameras measurements');
rms_err_yaw_acc = sqrt(sum((filt_yaw_acc-yaw_acc(1:length(filt_yaw_acc))).^2)/length(filt_yaw_acc))

acc = (filt_vel(2:end)-filt_vel(1:end-1))./(time(2:length(filt_vel))-time(1:length(filt_vel)-1));
filt_acc = test_filter(acc-acc(1),70,5)+acc(1);
figure; plot(spl,acc(spl),'LineWidth',0.85); hold on; grid on; plot(spl,filt_acc(spl),'LineWidth',0.85); legend('Acceleration computed with actuator measurements','Filtered signal'); xlabel('Sample'); ylabel('Yaw acceleration [rad/s^2]'); title('Acceleration computations from actuator measurements');
rms_err_acc = sqrt(sum((filt_acc-acc(1:length(filt_acc))).^2)/length(filt_acc))

%% IDENTIFICATION
Y_act = [filt_acc(1:end)',filt_vel(1:length(filt_acc))',(sign(filt_vel(1:length(filt_acc))))'];
res_act=lsqr(Y_act,(new_meas_eff(1:length(filt_acc)))')
J_act=res_act(1); C_act=res_act(2); Fric_act = res_act(3);

Y_cam = [filt_yaw_acc(1:end)',filt_yaw_vel(1:length(filt_yaw_acc))',(sign(filt_yaw_vel(1:length(filt_yaw_acc))))'];
res_cam=lsqr(Y_cam,(filt_trq(1:length(filt_yaw_acc)))')
J_cam=res_cam(1); C_cam=res_cam(2); Fric_cam = res_cam(3);

model_act = 1*J_act*filt_acc+C_act*filt_vel(1:length(filt_acc))+Fric_act*sign(filt_vel(1:length(filt_acc)));
model_cam = J_cam*filt_yaw_acc+C_cam*filt_yaw_vel(1:length(filt_yaw_acc))+Fric_cam*sign(filt_vel(1:length(filt_yaw_acc)));

final_rms_act = sqrt(sum((model_act-meas_eff(1:length(model_act))).^2)/length(model_act))
final_rms_cam = sqrt(sum((model_cam-meas_eff(1:length(model_act))).^2)/length(model_cam))

figure; plot(time,meas_eff); hold on; grid on; plot(time(1:length(model_act)),model_act); legend('Measured torque', 'Model based on actuator measurements');title('Model based on actuator measurements')
figure; plot(time,meas_eff); hold on; grid on; plot(time(1:length(model_act)),model_cam); legend('Measured torque', 'Model based on camera measurements');title('Model based on camera measurements')
figure; plot(time,meas_eff,'LineWidth',1.25); hold on; grid on; plot(time(1:length(model_act)),model_act,'LineWidth',1.25); plot(time(1:length(model_act)),model_cam,'LineWidth',1.25);legend('Measured torque', 'Model based on actuator measurements', 'Model based on camera measurements');title('Model prediction vs experimental measurements');xlabel('Time [s]'); ylabel('Torque [Nm]');
rms(model_act-model_cam)
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
