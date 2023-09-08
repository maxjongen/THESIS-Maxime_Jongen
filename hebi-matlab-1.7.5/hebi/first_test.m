%% Setup
clear *;
close all;
HebiLookup.initialize();

familyName = 'X8-16';
moduleNames1 = 'Drum'; 
group1 = HebiLookup.newGroupFromNames( familyName, moduleNames1 );
moduleNames2 = 'Arm'; 
group2 = HebiLookup.newGroupFromNames( familyName, moduleNames2 );
moduleNames3 = 'Base'; 
group3 = HebiLookup.newGroupFromNames( familyName, moduleNames3 );

cmd = CommandStruct();

duration = 2; % [sec]

%% Drum
timer = tic();

while toc(timer) < duration
    fbk = group1.getNextFeedback(); 
    cmd.velocity = 1.5;
    group1.send(cmd);
end
%% Arm
timer = tic();
while toc(timer) < duration
    fbk = group2.getNextFeedback(); 
    cmd.velocity = 0.1;
    group2.send(cmd);
end
%% Base
timer=tic();
while toc(timer) < duration
    fbk = group3.getNextFeedback(); 
    cmd.velocity = 0.2;
    group3.send(cmd);
end
