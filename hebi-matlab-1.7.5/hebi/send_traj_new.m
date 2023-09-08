% Send position commands, log in the background, and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% June 2018

%% Setup
clear all; clc;



family = '*'; % any family

group = HebiLookup.newGroupFromNames(family, 'Drum'); 
%disp(group);

%% Open-Loop Controller (Position)
% The command struct has fields for position, velocity, and effort.  

% Fields that are empty [] or NaN will be ignored when sending.
cmd = CommandStruct(); 

% Starts logging in the background
group.startLog( 'dir', 'logs' );  
fbk = group.getNextFeedbackFull();
% Parameters for sin/cos function
freqHz = 0.15;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 10 );    % [rad]
q0 = fbk.position;
duration = 20; % [sec]
% fbk = group.getNextFeedbackFull();
% q0 = fbk.position;
% 
% qf = q0 + deg2rad(90);
% Ts=1e-2;
% 
% t1=0:Ts:duration;
% 
% %Generate the trajectory (Robotics toolbox)
% [q1,qd1,qdd1]=my_traj(q0,qf,t1);
Ts = 0.01;
fs = 1/Ts; %in Hz
fc = 5*freqHz; % cutoff frequency
% [b,a] = butter(1,fc/(fs/2),'low');
% prev_vel = 0;
% prev_filt_vel = 0;

filter_order = 13;      % Filter order (odd number)

% Convert cutoff frequency from Hz to normalized units
fc_normalized = fc / (fs/2);

% Design FIR filter using windowing method
filter_coeffs = fir1(filter_order, fc_normalized, 'low');

% Initialize filter buffer
vel_buffer = zeros(filter_order, 1);
vel_err_buffer = zeros(filter_order, 1);


i=1;

timer = tic();

while toc(timer) < duration
    
    t = toc(timer);
    fbk = group.getNextFeedbackFull();
    % Position command
    cmdPosition = q0-amp+amp * cos( freq * t );

    % Velocity command (time-derivative of position)
    cmdVelocity = -freq * amp * sin( freq * t );

%     cmdPosition = q1(i);
% 
%     % Velocity command (time-derivative of position)
%     cmdVelocity = qd1(i);

    % Update set points
    cmd.position = cmdPosition;
    cmd.velocity = cmdVelocity;
    %cmd.effort = -4.21*cmdPosition - 0.5*0.022;
    group.send(cmd);  
    pos(i) = fbk.position;
    vel(i) = fbk.velocity;
    
%     filtered_vel(i) = b(1)*vel(i) + b(2)*prev_vel - 0*a(2)*prev_filt_vel;
%     prev_vel = vel(i);
%     prev_filt_vel = filtered_vel(i);

    vel_buffer(2:end) = vel_buffer(1:end-1);

    % Add new input sample to buffer
    vel_buffer(1) = vel(i);

    % Apply filter to input buffer
    %y = filter(filter_coeffs, 1, vel_buffer(1));
    y = filter_coeffs(1:end-1)*vel_buffer;
    filtered_vel(i) = y;
    err_vel(i) = -freq * amp * sin( freq * t ) - filtered_vel(i);

    vel_err_buffer(2:end) = vel_err_buffer(1:end-1);

    % Add new input sample to buffer
    vel_err_buffer(1) = err_vel(i);

    err_vel_filt(i) = filter_coeffs(1:end-1)*vel_err_buffer;

    meas_eff(i) = fbk.effort;
    windingCurrent(i) = fbk.windingCurrent;
    
    
    pwm(i) = fbk.pwmCmd;
    time(i)=t;
    i = i+1;
    
end

% Stop logging and plot the position data using helper functions
trq_th = -1.2*freq^2*amp*cos(freq*time);
g_compens = -4.21*pos-0.022;
% acc = 3.2*qdd1;
log = group.stopLog();
HebiUtils.plotLogs( log, 'position' );
% figure; plot(time,meas_eff); hold on;  plot(time,trq_th); title('Torque from the actuator'); 
% legend('Measured torque','Theoretical torque'); xlabel('Time [s]'), ylabel('Torque [Nm]'); grid on;

figure; plot(time,vel); hold on;  plot(time,filtered_vel); plot(time,-freq * amp * sin( freq * time )); title('Real-time velocity measurements'); 
legend('Unfiltered measurements','Filtered measurements', 'Command'); xlabel('Time [s]'), ylabel('Velocity [rad/s]'); grid on;
