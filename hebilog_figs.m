%% Figures for hebilog files
close all

figure;
effort = test_filter(effort,20,5);
plot(time(1:length(effort)),effort,'LineWidth',0.75); grid on; xlabel('Time [s]'); ylabel('Measured torque [Nm]'); title('Measured effort for the unloaded "Base" actuator')

figure;

plot(time,position-position(1)+0.73,'LineWidth',1.2); grid on; xlabel('Time [s]'); ylabel('Measured position [rad]'); title('Measured position for the unloaded "Base" actuator')
