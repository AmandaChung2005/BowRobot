function [error] = singleModeMeanError(x, tf_dB, fs ,modeWidth)
% Calculate the mean square error with the frequency band scaling
%

    % freqLimits is set so the optimization only occurs in a certain
    % frequency band, this could be changed as well

    N = length(tf_dB); % Gets the length of the passed in transfer function (in dB)
    dur = N/fs;

    % For the optimization, the variables (fo, gamma, Q) need to be passed
    % as a vector (x)
    freq = x(1);
    dr = x(2);
    amp = x(3);


    % Get the simulated mode
    [irhat, t] = modalIr(freq,dr,amp,fs,dur);
                             
    H_dB = 20*log10(abs(fft(irhat)));
    

    % Get the frequency limit indices
    errorFreqLimitsIndex = [round(freq)-modeWidth,round(freq)+modeWidth];


    % Error metric is the mean absolute error difference, you could to
    % change this to try other error metrics
    error = mean(abs(tf_dB(errorFreqLimitsIndex(1):errorFreqLimitsIndex(2)) - H_dB(errorFreqLimitsIndex(1):errorFreqLimitsIndex(2))));


end
