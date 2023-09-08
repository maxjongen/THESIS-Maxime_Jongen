% This code is largely based on Christophe and Art's wor

% TO BE RUN BY SECTIONS

% Each section allows to create different types of trajectrories
% The last section allows to send the trajectory to the motor(s)

%% RUN THIS SECTION (ONCE) FIRST
clear all; clc;

family = '*'; % any family

group = HebiLookup.newGroupFromNames(family, 'Drum'); 
%disp(group);


% Bypass HEBI control strategies
gains = GainStruct();
gains.controlStrategy = ones(1,group.getNumModules)*2; %Use of strategy (*1 strategy 1, *2 strategy 2, *3 strategy 3, *4 strategy 4)
gains.positionKp = [0]; %set gains according to strategy (see list of gains/strategy)
gains.velocityKp = [0];
gains.effortKp = [0.1]*0.75;
gains.effortKd = [0.1]*0;

group.send('gains', gains);

% set KP, KI, KD of the PID controller developed
% kP =[100]/0.35;%[80  80];%[15 30 15]; %130
kP =[100]/1.2;%[80  80];%[15 30 15]; %130
kI = 0.1*0+0;
kD = 0.1*1;
% kD = 0.1*0.5;

%% Bring back to zero position
clear time; clear command; clear effort; clear pos; clear vel; clear pwm;
time = [];
pos = [];
vel_err_vec = [];
vel_err2_vec = [];

cmd = CommandStruct();
i=1;
last_err = 0;

feedback = group.getNextFeedback();%Reed all sensor data
q0 = feedback.position;

qf = 0;
tf=[3]; %Set time of traj
Ts=1e-2;

t1=0:Ts:tf(1);

%Generate the trajectory (Robotics toolbox)
[q1,qd1,qdd1]=jtraj(q0(1),qf(1),t1);

N = length(q1);
i=1;

timer = tic();
pause(0.01)
while i<=N
    
    fbk = group.getNextFeedbackFull();
    pos(i) =  fbk.position;
    vel(i) =  fbk.velocity;
    
    pos_err = q1(i) - fbk.position;
    pos_err_vec(i) = pos_err;
    
    vel_err = qd1(i) - fbk.velocity;
    vel_err_vec(i) = vel_err;
    
    vel_err2 = (pos_err - last_err)/Ts;
    vel_err2_vec(i) = vel_err2;
    last_err = pos_err;
    
    cmd_eff = kP*pos_err + kD*vel_err2;
    cmd.effort = 1*cmd_eff;
    effort(i) = cmd_eff;
    pwm(i) = fbk.pwmCmd;
    group.send(cmd); %send the command to the motor
    time(i)=toc(timer);
    i = i+1;
    %pause(Ts);
end

figure

plot(time,q1)
hold on
plot(time,pos)
legend('Command', 'pos')

title('Position')

figure;
plot(time,effort)
title('Effort')


%% Send sinusoidal trajectory
close all;
clear time; clear command; clear effort; clear pos; clear vel;
time = [];
pos = [];
vel_err_vec = [];
vel_err2_vec = [];
fbk = group.getNextFeedbackFull();
q0 = fbk.position;
cmd = CommandStruct();
i=1;
last_err = 0;

freqHz = 0.15;           % [Hz]

freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 30 );    % [rad]


tf=[20]; % Approximate duration of the trajectory
Ts=1e-2;

t1=0:Ts:tf(1);
% q1 = amp * cos( freq * t1 )-amp; %starting at 0 !!!
% qd1 = -freq * amp * sin( freq * t1);
N = length(t1);
last_t = 0;

timer = tic();
pause(0.01)
while i<=N
    
    t = toc(timer);
    q1 = q0+amp * cos( freq * t )-amp; %starting at 0 !!!
    command(i) = q1;
    fbk = group.getNextFeedbackFull();
    pos(i) =  fbk.position;
    vel(i) =  fbk.velocity;
    
    pos_err = q1 - fbk.position;
    pos_err_vec(i) = pos_err;
    
    vel_err2 = (pos_err - last_err)/(t-last_t);
    vel_err2_vec(i) = vel_err2;
    last_err = pos_err;
    
    cmd_eff = 1*(kP*pos_err + kD*vel_err2);
    cmd.effort = 1*cmd_eff -0.7*4.21*pos(i) - 0.7*0.022;
    effort(i) = cmd_eff;
    meas_eff(i) = fbk.effort;
    pwm(i) = fbk.pwmCmd;
    motorCurrent(i) = fbk.motorCurrent;
    windingCurrent(i) = fbk.windingCurrent;
    group.send(cmd); %send the command to the motor
    last_t = t;
    time(i)= t;
    i = i+1;
    %pause(Ts);
end

figure
plot(time,command)
hold on
plot(time,pos)
legend('Command', 'pos')
title('Position')


figure;
plot(effort)
title('Effort')


%% Constant speed reference (position ramp)
close all;
clear time; clear command; clear effort; clear pos; clear vel;
time = [];
pos = [];
vel_err_vec = [];
vel_err2_vec = [];

cmd = CommandStruct();
i=1;
last_err = 0;
feedback = group.getNextFeedback();%Reed all sensor data
q0 = feedback.position;

desired_vel = 0.15;           % [Hz]

tf=[5]; % Approximate duration of the trajectory
Ts=1e-2;

t1=0:Ts:tf(1);

N = length(t1);
last_t = 0;

timer = tic();
pause(0.01)
while i<=N
    
    t = toc(timer);
    q1 = q0+desired_vel*t; %starting at 0 !!!
    command(i) = q1;
    fbk = group.getNextFeedbackFull();
    pos(i) =  fbk.position;
    vel(i) = fbk.velocity;
    
    pos_err = q1 - fbk.position;
    pos_err_vec(i) = pos_err;
    
    vel_err2 = (pos_err - last_err)/(t-last_t);
    vel_err2_vec(i) = vel_err2;
    last_err = pos_err;
    
    cmd_eff = kP*pos_err + kD*vel_err2;
    effort(i) = cmd_eff;
    meas_eff(i) = fbk.effort;
    windingCurrent(i) = fbk.windingCurrent;
    
    cmd.effort = cmd_eff;
    
    pwm(i) = fbk.pwmCmd;
    
    volt(i) = fbk.voltage;
    
    group.send(cmd); %send the command to the motor
    last_t = t;
    time(i)= t;
    i = i+1;
    pause(Ts);
end

figure
plot(time,command)
hold on
plot(time,pos)
legend('Command', 'pos')
title('Position')


figure;
plot(time,effort)
title('Effort')

%% Impose torque
close all;
clear time; clear command; clear effort; clear pos; clear vel;clear pwm;
time = [];
pos = [];
vel_err_vec = [];
vel_err2_vec = [];

cmd = CommandStruct();
i=1;
last_err = 0;

tf=[6]; % Approximate duration of the trajectory
Ts=1e-2;

t1=0:Ts:tf(1);

N = length(t1);
last_t = 0;

timer = tic();
pause(0.01)
while i<=N
    
    t = toc(timer);
    fbk = group.getNextFeedbackFull();
    
    pos(i) =  fbk.position;
     vel(i) = fbk.velocity;
    cmd_eff = 1;
    cmd.effort = cmd_eff;
    effort(i) = cmd_eff;
    pwm(i) = fbk.pwmCmd;
    volt(i) = fbk.voltage;
    motorCurrent(i) = fbk.motorCurrent;
    windingCurrent(i) = fbk.windingCurrent;
    mes_eff(i) = fbk.effort;
%     vel_lim(i) = fbk.velocityLimitState;
    group.send(cmd); %send the command to the motor
    time(i)= t;
    i = i+1;
end


figure;
plot(time,pos)
title('Position')

figure;
plot(time,windingCurrent)
title('Current')