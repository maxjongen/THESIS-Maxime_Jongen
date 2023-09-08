% Define filter coefficients
fc = 0.1;           % Cutoff frequency
fs = 1000;          % Sampling frequency
[b,a] = butter(1,fc/(fs/2),'low');    % Butterworth filter design

% Initialize variables
y = 0;              % Filtered output
x_prev = 0;         % Previous input sample

% Start real-time filtering loop
while true
    % Acquire input sample (replace this with your own acquisition code)
    x = <your input acquisition code>;

    % Apply filter to input sample
    y = b(1)*x + b(2)*x_prev - a(2)*y;

    % Store previous input sample for next iteration
    x_prev = x;

    % Output filtered sample (replace this with your own output code)
    <your output code>;
end