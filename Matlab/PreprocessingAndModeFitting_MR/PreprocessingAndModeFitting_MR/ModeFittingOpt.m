function [fmhat, drhat, gmhat, irhat] = ModeFittingOpt(ir,fs,dur)
% May 2024
% Mark Rau
%
% Get the modal parameters (gain, resonance frequency, and damping) of an
% impulse response with optimization and peak picking
%
% ir = impulse response
% fs = sample rate
% len = length of signal to analyse

N = fs*dur;
Admitt = fft(ir(1:N));

%% Initial mode guess
% freqLimsLow = [80:150];
% freqLimsMid = [160:300];
% freqLimsHigh = [300:1000];

freqLimsLow = [500:700];
freqLimsMid = [700:1500];
freqLimsHigh = [1500:2000];

Admitt_dB = 20*log10(abs(Admitt));

% [peaksLow,idxLow] = findpeaks(Admitt_dB(freqLimsLow),'MinPeakHeight',-50,'MinPeakDistance',55,'MinPeakProminence',10);
% [peaksMid,idxMid] = findpeaks(Admitt_dB(freqLimsMid),'MinPeakHeight',-50,'MinPeakDistance',5,'MinPeakProminence',5);
% [peaksHigh,idxHigh] = findpeaks(Admitt_dB(freqLimsHigh),'MinPeakHeight',-60,'MinPeakDistance',2,'MinPeakProminence',2);

[peaksLow,idxLow] = findpeaks(Admitt_dB(freqLimsLow),'MinPeakHeight',-30, 'MinPeakDistance', 2, 'MinPeakProminence', 2);
[peaksMid,idxMid] = findpeaks(Admitt_dB(freqLimsMid),'MinPeakHeight',-30,'MinPeakDistance',2,'MinPeakProminence',2);
[peaksHigh,idxHigh] = findpeaks(Admitt_dB(freqLimsHigh),'MinPeakHeight',-30,'MinPeakDistance',2,'MinPeakProminence',2);

peakFreqs = [idxLow+freqLimsLow(1)-2; idxMid+freqLimsMid(1)-2; idxHigh+freqLimsHigh(1)-2];
peakAmps = 10.^([peaksLow; peaksMid; peaksHigh]/20);

numModes = length(peakFreqs);
freqs_g1 = peakFreqs;
dr_g1 = 0.01*ones(numModes,1);
% amps_g1 = peakAmps.*ones(numModes,1)/1000;
amps_g1 = 2*peakAmps;


%% First optimiziation, individual modes
freqs_g2 = zeros(numModes,1);
dr_g2 = zeros(numModes,1);
amps_g2 = zeros(numModes,1);

modeWidth = 5;

for k = 1:numModes

    x0 = [freqs_g1(k), dr_g1(k), amps_g1(k)]; % Need to make the variables as a vector for the optimization

    % Define a function which returns the error at the current "solution"
    fun = @(x)singleModeMeanError(x, Admitt_dB, fs ,modeWidth);
    
    % lower bounds for the variables. This is a constrained optimization, so
    % you can set upper and lower bounds for each variable.
    lb = [freqs_g1(k)-3,0.001,0];
    ub = [freqs_g1(k)+3,0.1,1];
    
    % Options for the optimization, you could change these and test different
    % things if you want
    options = optimoptions('fmincon', 'OptimalityTolerance', 1e-10, 'MaxIterations', 5e3, 'MaxFunctionEvaluations', 5e4, 'StepTolerance',1e-100,'Display', 'off');
    
    % Run the optimiuzation here
    [x,fval,exitflag,output] = fmincon(fun,x0,[],[],[],[],lb,ub,[], options);

    freqs_g2(k) = x(1);
    dr_g2(k) = x(2);
    amps_g2(k) = x(3);

end


%% Second Optimization, all modes, just amplitudes
x0 = amps_g2; % Need to make the variables as a vector for the optimization

% Define a function which returns the error at the current "solution"
% freqLimits = [75,1000];
freqLimits = [75,4000];
fun = @(x)allModesMeanError_justAmps(x, freqs_g2, dr_g2, Admitt_dB, fs ,freqLimits);

% lower bounds for the variables. This is a constrained optimization, so
% you can set upper and lower bounds for each variable.
lb = zeros(length(amps_g2),1);
ub = [freqs_g1(k)+1,0.1,1];

% Options for the optimization, you could change these and test different
% things if you want
options = optimoptions('fmincon', 'OptimalityTolerance', 1e-10, 'MaxIterations', 5e3, 'MaxFunctionEvaluations', 5e4, 'StepTolerance',1e-100,'Display', 'off');

% Run the optimiuzation here
[x,fval,exitflag,output] = fmincon(fun,x0,[],[],[],[],lb,[],[], options);

freqs_g3 = freqs_g2;
dr_g3 = dr_g2;
amps_g3 = x;


%% Third Optimization, all modes, all params
x0 = [freqs_g3, dr_g3, amps_g3]; % Need to make the variables as a vector for the optimization

% Define a function which returns the error at the current "solution"
% freqLimits = [75,1000];
freqLimits = [75,1000];
fun = @(x)allModesMeanError_allParams(x, Admitt_dB, fs ,freqLimits);

% lower bounds for the variables. This is a constrained optimization, so
% you can set upper and lower bounds for each variable.
lb = [x0(:,1)-2, 0.5.*x0(:,2) ,0.*x0(:,3)];
ub = [x0(:,1)+2, 2.*x0(:,2) ,2.*x0(:,3)];

% Options for the optimization, you could change these and test different
% things if you want
options = optimoptions('fmincon', 'OptimalityTolerance', 1e-10, 'MaxIterations', 5e3, 'MaxFunctionEvaluations', 5e4, 'StepTolerance',1e-100,'Display', 'off');

% Run the optimiuzation here
[x,fval,exitflag,output] = fmincon(fun,x0,[],[],[],[],lb,ub,[], options);

freqs_g4 = x(:,1);
dr_g4 = x(:,2);
amps_g4 = x(:,3);


fmhat = freqs_g4;
drhat = dr_g4;
gmhat = amps_g4;

[irhat4, t] = modalIr(freqs_g4,dr_g4,amps_g4,fs,dur);
irhat = irhat4;

%% Debugging
disp('peakFreqs')
disp(sort(peakFreqs))

disp('freqs_g2')
disp(sort(freqs_g2))

disp('freqs_g3')
disp(sort(freqs_g3))

disp('fmhat')
disp(sort(fmhat))


end