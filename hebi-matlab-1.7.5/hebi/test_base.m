%% Setup
clear *;
close all;
HebiLookup.initialize();

familyName = 'X8-16';

moduleNames3 = 'Base'; 
group3 = HebiLookup.newGroupFromNames( familyName, moduleNames3 );

cmd = CommandStruct();

duration = 2; % [sec]

%% Base
timer=tic();
while toc(timer) < duration
    fbk = group.getNextFeedback(); 
    cmd.velocity = -0.3;
    group3.send(cmd);
end
