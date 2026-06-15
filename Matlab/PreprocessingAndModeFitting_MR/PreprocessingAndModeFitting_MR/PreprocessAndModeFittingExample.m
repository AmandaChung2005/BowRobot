% Example of admittance preprocessing and mode fitting
%
% Mark Rau
% July 2024
%
% The mode fitting works with three optimization steps. 
% First, peak picking is done to find the most prominent frequency peaks.
% Then the frequency, damping ratios, and amplitude of each mode is fit
% individually.
% Secondly, an optimization is done on all modes, but just on the
% amplitudes, do get a better amplitude guess taking into account the the mode interactions. 
% Finally, an optimization is done on all mode frequencies, damping ratios,
% and amplitudes to get a slightly better fit. 
%

close all
clear all
clc

%% Load and preprocess a measurement
plotBool=1; % 1 if you want to plot results, 0 if not

% Load an example ir
load('Nishihara_meas.mat')

fs = freq; % sample rate
dur = 1; % duration to keep
N = round(fs*dur); % number of samples to keep

hammerThresh = 0.02; % threshold ratio of max value to keep for the hammer strike
hammerPhase = 1; % phase of the hammer strike (if inverted, make -1)
BeginAtMax = 0; % if you want the measurement to start from the hammer peak, set to 1, 0 otherwise


% Make a struct called "meas" that will hold the variables
meas.force = indata(:,1);
meas.velocity = indata(:,2);


[sig] = CalculateCleanIR(meas,fs,N,hammerThresh,hammerPhase,BeginAtMax);



guitarAdmitt = sig.admittance; % guitar admittance time domain
GuitarAdmitt = fft(guitarAdmitt); % guitar admittance frequency domain



% Plot results
if(plotBool==1)
    f = (0:N-1)*fs/N;
    t = (0:N-1)/fs;

    figure
    subplot(3,1,1)
    plot(t(1:50),sig.forceShift(1:50),"LineWidth",2)
    xlabel('Time (s)')
    ylabel('Force (N)')
    grid on
    title('Force Hammer Domain')

    subplot(3,1,2)
    plot(t,sig.velocityShift,"LineWidth",2)
    xlabel('Time (s)')
    ylabel('Velocity (m s^{-1})')
    grid on
    title('LDV Time Domain')

    subplot(3,1,3)
    semilogx(f,20*log10(abs(fft(sig.forceShift))),"LineWidth",2);
    hold on;
    semilogx(f,20*log10(abs(fft(sig.velocityShift))),"LineWidth",2);
    semilogx(f,20*log10(abs(GuitarAdmitt)),"LineWidth",2);
    title('Frequency Domain')
    xlim([50,20000])
    ylim([-80,60])
    xlabel('Frequency (Hz)')
    ylabel('Magnitude (dB)')
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', [300 100 1200 600]);
    legend('Force (N)','Velocity (m s^{-1})','Admittance ((m s^{-1} N^{-1}))')
end






%% Mode fitting
plotBool=1; % 1 if you want to plot results, 0 if not

tic
[fmhat, drhat, gmhat, irhat] = ModeFittingGuitarOpt(guitarAdmitt,fs,dur);
toc


% Plot results
if(plotBool==1)
    f = (0:N-1)*fs/N;
    figure
    dur = 1;
    [irhat, t] = modalIr(fmhat,drhat,gmhat,fs,dur);

    semilogx(f,20*log10(abs(GuitarAdmitt)),"LineWidth",2);
    hold on;
    semilogx(f,20*log10(abs(fft(irhat))),"LineWidth",2);
    title('Mode Fitting: Guitar')
    xlim([50,1000])
    ylim([-100,0])
    xlabel('Frequency (Hz)')
    ylabel('Admittance (dB) (m s^{-1} N^{-1})')
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', [300 100 1200 600]);
    legend('Measurement','Fit')
end



