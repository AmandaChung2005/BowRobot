 function delta_HMD = HMD_example(popn1, popn2, conf)
% INPUTS:
% vector popn1, measurement values in first population 
% vector popn2, measurement values in second population
% scalar confidence level (default is 0.95)
% OUTPUTS
% hypothesized mean difference at specified confidence level

% If no confidence level is given, use 0.95 confidence
if nargin == 2
    conf = 0.95;
end

% Set things up such that popn1 has greater mean to use one-sided t-test
if mean(popn1) < mean(popn2)
    temp = popn1;
    popn1 = popn2;
    popn2 = temp;
end
 
n1 = length(popn1);
mu1 = mean(popn1);
sd1 = std(popn1);

n2 = length(popn2);
mu2 = mean(popn2);
sd2 = std(popn2);

se12 = sqrt(sd1^2/n1 + sd2^2/n2);

% Calculate dof for Welch's test: 
% Ref: https://en.wikipedia.org/wiki/Welch%27s_t-test
df = floor( se12^4 / (((sd1^2/n1)^2)/(n1-1) + ((sd2^2/n2)^2)/(n2-1)) );

t_factor = tinv(1-(1-conf)/2, df); %for 2-tailed t-test you need to decrease
% the p-value by a factor of two (from 0.05 to 0.025 if using 2.671
% defaults) to account for the fact that the result might be in the other
% tail 

% t_factor = tinv(1-(1-conf), df); % uncomment for 1-tailed t-test
%for 1-tailed t-test you use the desired p-value (0.05 for 2.671 default) 
% because you know a priori that the result can only be in one tail. 

delta_HMD = mu1 - (mu2 + t_factor*se12);

end