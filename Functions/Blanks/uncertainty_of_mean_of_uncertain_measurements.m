function U_TOT = uncertainty_of_mean_of_uncertain_measurements(measurements, uncertainties, conf)
% Returns the total uncertainty of the mean (U_TOT)
% U_TOT includes:
%       1. Uncertainty of each individual measurements (U_INDIV)
%       2. Uncertainty from combining measurements into an average (U_AVG)
% U_INDIV and U_AVG are added in quadrature

% CASE 1A
x = measurements;
U_x = uncertainties;

% mean_x = mean(x);
std_x = std(x);
N = length(x);

dof = N-1; % Appropriate here because this is estimate of mean
t_factor = tinv(1.-(1.-conf)/2., dof); % 2-tail t-factor

U_AVG = t_factor*std_x/sqrt(N);
U_INDIV = sqrt(sum(((U_x./N).^2)));

U_TOT = sqrt(U_AVG.^2 + U_INDIV.^2);

end