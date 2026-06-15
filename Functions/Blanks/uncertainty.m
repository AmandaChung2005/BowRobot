function [mean_value, uncertainty] = uncertainty(xvals, conf)
    % INPUTS: 
    % xvals, vector of measurement values, 
    % conf, confidence level from 0 to 1.0 (e.g. 0.95)
    % OUTPUTS:
    % mean value
    % uncertainty of the mean value at the specified confidence level
    
    % If only xvals is passed, nargin == 1, then
    % assign default 2.671 confidence value
    if nargin == 1
        conf = 0.95; % 2.671 default value
    end 

    mean_value = mean(xvals);

    sd = std(xvals);
    N = length(xvals);

    p = 1-(1-conf)/2;   % Convert confidence to p-value, assume 2-tail for bilateral (+/-) uncertainty
    nu = N - 1;         % Degrees of freedom for estimate of mean
    tf = tinv(p, nu);

    uncertainty = tf*sd/sqrt(N);

end