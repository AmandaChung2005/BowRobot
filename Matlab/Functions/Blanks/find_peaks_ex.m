close all;
% Make some data that I know will have peaks
t = 0:0.01:100.0; t=t(:);
dt = mean(diff(t));
Fs = 1.0/dt;
y = 20.*randn(length(t),1);
for i = 1:5
    f = 5*i;
    y = y + 2.0*i*sin(2*pi*f*t);
end


[power, freq] = pwelch(y, [], [] , [], Fs);
% [power, freq] = periodogram(y, [], [], Fs);

% Compute the integral of the PSD approximation
% trapz(freq, power)


% Find 4 largest peaks
for N = [4, ]
    [pks, x_locs] = find_N_peaks(freq, power, N, 1);
    title(sprintf('%d peaks', N));
end
xlabel('Frequency (Hz)');
ylabel('PSD Amplitude');
%%

% for f = [100.0, 100.5, 101.0]
f = 101.0;
t = 0:0.001:1.0;
Fs = 1/mean(diff(t));
y = 3.5*sin(2*pi*f*t);

[power, freq] = periodogram(y, [], [], Fs);


N = 1;
    [pks, x_locs] = find_N_peaks(freq, power, N, 1);
    title(sprintf('%d peaks', N));
xlabel('Frequency (Hz)');
ylabel('PSD Amplitude');

% end

