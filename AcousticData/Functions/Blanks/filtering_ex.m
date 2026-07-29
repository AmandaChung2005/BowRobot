close all; clear all; clc;

% Make up some data to play with
% You can replace this section with code to load your own t, y data
t = 0:0.001:10.0; t=t(:);
Fs = 1.0/mean(diff(t));
f1 = 10.0;
fnoise = 60.;

y = sin(2*pi*f1*t) + 0.5*sin(2*pi*fnoise*t) + 0.1*randn(length(t), 1);

%%
% Plot the raw data
figure(1);
subplot(2,2,1);
plot(t, y, 'DisplayName', 'Raw data');
xlabel('Time (s)');
ylabel('Amplitude (V)');
title('Raw data');
xlim([0, 0.5]);
improvePlot;

%%
% Plot the PSD
[power, freq] = pwelch(y, [], [] , [], Fs);
% figure(2);
subplot(2,2,2);
plot(freq, power);
xlabel('Frequency (Hz)')
ylabel('Power (V^2/Hz)');
xlim([1, 100]);
title('PSD of raw data');
improvePlot;


%%
% Create a low-pass filter to delete the 60 Hz noise
fc = 40.;
filter_order = 10;
[b,a] = butter(filter_order, fc/(Fs/2.), 'low');

% Apply the filter with filt-filt to avoid phase delay
yfilt = filtfilt(b, a, y); % The vector yfilt is a filtered version of y

% Superimpose the filtered signal on the original plot
subplot(2,2,3);
hold on;
plot(t, yfilt, 'r-', 'LineWidth', 4);
xlabel('Time (s)');
ylabel('Amplitude (V)');
title('Filtered data');
xlim([0, 0.5]);
improvePlot;

% Plot the new PSD
[power, freq] = pwelch(yfilt, [], [] , [], Fs);
subplot(2,2,4);
plot(freq, power, 'r-');
xlabel('Frequency (Hz)')
ylabel('Power (V^2/Hz)');
xlim([1, 100]);
title('PSD of filtered data');
improvePlot;