function [error] = allModesMeanError_allParams(x, tf_dB, fs ,freqLimits)
% Calculate the mean square error with the frequency band scaling
%

    % freqLimits is set so the optimization only occurs in a certain
    % frequency band, this could be changed as well

    N = length(tf_dB); % Gets the length of the passed in transfer function (in dB)
    dur = N/fs;

    % For the optimization, the variables (fo, gamma, Q) need to be passed
    % as a vector (x)
    freqs = x(:,1);
    dr = x(:,2);
    amps = x(:,3);


    % Get the simulated mode
    [irhat, t] = modalIr(freqs,dr,amps,fs,dur);
                             
    H_dB = 20*log10(abs(fft(irhat)));
    

    % Get the frequency limit indices
    errorFreqLimitsIndex = round(N*freqLimits/fs);


    % Error metric is the mean absolute error difference, you could to
    % change this to try other error metrics
%     error = mean(abs(tf_dB(errorFreqLimitsIndex(1):errorFreqLimitsIndex(2)) - H_dB(errorFreqLimitsIndex(1):errorFreqLimitsIndex(2))));

    % Try to weight the error metric
    errorFreqPeaks = mean(abs(tf_dB(round(freqs+1)) - H_dB(round(freqs+1))));
    errorMean = mean(abs(tf_dB(errorFreqLimitsIndex(1):errorFreqLimitsIndex(2)) - H_dB(errorFreqLimitsIndex(1):errorFreqLimitsIndex(2))));

   

    % Peak matching error
    df= fs/N;
    freqBins= round(freqs./df)+1;
    peakError= 0;

    for k=1:length(freqBins)
        idx= max(1, freqBins(k)-3):min(N, freqBins(k)+3);
        peakError= peakError+mean(abs(tf_dB(idx)-H_dB(idx)));
    end

    peakError= peakError/length(freqBins);

    % Total Error
    error = errorMean + 100*errorFreqPeaks;

    disp('Measured peak heights')
disp(tf_dB(freqBins))

disp('Model peak heights')
disp(H_dB(freqBins))
end
