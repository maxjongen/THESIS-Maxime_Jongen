function filtered_signal = test_filter(signal,n_gauss,alpha)

% %Moving average smoothing filter
% a = 1;                      %Denominator of the TF of the filter
% avnum = 20;                  %Size of the filter
% b = 1/avnum*ones(1,avnum);  %Numerator of the filter

% y = filter(b,a,x);

w = gausswin(n_gauss, alpha);
w = w/sum(w);
filtered_signal = filter(w, 1, signal);

filtered_signal = filtered_signal(round(n_gauss/2):end);

end