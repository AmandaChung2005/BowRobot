% Test Optimization Modal Fitting
%
% Mark Rau
% May 2024
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

%% Load and get an IR
% Load an example ir
data= load('guitarAdmitt.mat');
data1= load('CarbonFiberFromBenoit1_clamped_meas1');

% signal= data.guitarAdmitt;
signal= data1.indata(:,2);

%%
fs = 4800;
N = length(signal);
plotBool=1;
dur = N/fs;

signal_fft= fft(signal(1:N));

% signalPadded= [signal; zeros(fs-length(signal),1)];
% signal_fft = fft(signalPadded(1:N));


% Mode fitting
tic
[fmhat, drhat, gmhat, irhat] = ModeFittingOpt(signal,fs,dur);
% [fmhat, drhat, gmhat, irhat] = ModeFittingOpt(signalPadded,fs,dur);
toc


% Plot results
if(plotBool==1)
    f = (0:N-1)*fs/N;
    figure
    dur = N/fs;
    [irhat, t] = modalIr(fmhat,drhat,gmhat,fs,dur);

   
   hold on
   plot(f,20*log10(abs(signal_fft)))
   hold off



figure
    semilogx(f,20*log10(abs(signal_fft)),"LineWidth",2);
    hold on

    for k=1:length(fmhat)
        xline(fmhat(k),'r--')
    end

    semilogx(f,20*log10(abs(fft(irhat))),"LineWidth",2);
    title('Mode Fitting')
    xlim auto
    ylim auto
    xlabel('Frequency (Hz)')
    ylabel('Admittance (dB) (m s^{-1} N^{-1})')
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', [300 100 1200 600]);
    legend('Measurement','Fit')

   disp([fmhat gmhat])

   figure
stem(fmhat,gmhat)
xlabel('Frequency (Hz)')
ylabel('Modal amplitude')
end

