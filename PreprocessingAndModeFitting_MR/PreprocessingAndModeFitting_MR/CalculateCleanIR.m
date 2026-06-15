function [sig] = CalculateCleanIR(meas,fs,len,hammerThresh,hammerPhase,BeginAtMax)
% Takes in input and output signals for an IR stored as a structure array
% in meas. Does frequency domain deconvolution while cleaning up the
% signals.
%
% INPUTS
% meas = sytuct array of measurements having fields {hammer, LDV}
% fs = sample rate
% len = length of IR signal you want returned
% hammerThresh = threshold of the hammer force when it is deemed to go down
% to noise (in percentage of the max value)
% hammer phase = gives hammer phase inversion information (-1 if hammer
% strike is phase inverted, 1 otherwise)
%
% OUTPUTS
% sig = structure array which includes the admittance and shifted signals
% {hammer, LDV, force, velocity, admittance, forceShift, velocityShift}
%
% Specifically designed for hammer/LDV measurements


% Compensate for delay time of LDV if necessary
LDV_sampDelay = 0 ;

% Get the peak of the hammer strike
[peak, loc] = max(meas.force);


% Find the start of the hammer measurement
if BeginAtMax==1 
    lowerBound = loc;
    i=0;
else
    i = 1;
    foundLower = false;
    while(foundLower==false)
        if (meas.force(loc-i))<hammerThresh*peak && loc-i>1
            lowerBound = loc-i;
            foundLower = true;
        elseif loc-i == 1
            lowerBound = loc-i;
            foundLower = true;
        else
            i = i+1;
        end
    end
end

% Find the end of the hammer strike
j = 1;
foundUpper = false;
while(foundUpper==false)
    if (meas.force(loc+j))<hammerThresh*peak
        upperBound = loc+j;
        foundUpper = true;
    else
        j = j+1;
    end
end

% move the signals to start at time zero and get rid of noise.
hammer = zeros(len,1);
hammer(1:i+j+1) = meas.force(lowerBound:upperBound);

velocity = zeros(len,1);
% velocity(1:len) = filter(Hd,meas.velocity(lowerBound:lowerBound+len-1));
velocity(1:len) = meas.velocity(lowerBound:lowerBound+len-1);
velocity(1:len) = meas.velocity(lowerBound+LDV_sampDelay:lowerBound+len+LDV_sampDelay-1);


% Zero pad the signals
pow = nextpow2(len)+2;
hammerZeroPad = hammerPhase*[hammer;zeros(2^pow-len,1)];
% velocity = [velocity(LDV_sampDelay:end);zeros(LDV_sampDelay,1)];

velocityZeroPad = [velocity;zeros(2^pow-len-1,1)];


% Do the deconvolution for admittance
admittance = fftdeconv(velocityZeroPad',hammerZeroPad');    
% admittance = fftdeconv(velocityZeroPad',hammerZeroPad',10000/(fs/2),100/(fs/2));   
admittance = admittance(1:len).';


hammerZeroPad = hammerZeroPad(1:len);
velocityZeroPad = velocityZeroPad(1:len);


% Add the signals to the struct
sig = meas;
sig.admittance = admittance;
sig.forceShift = hammerZeroPad;
sig.velocityShift = velocityZeroPad;


end






