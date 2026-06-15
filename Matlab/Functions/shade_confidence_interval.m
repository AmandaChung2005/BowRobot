% Define the functional form of the fit
fit_type = fittype('a*x + b');

% Execute the fit
% You might need to change the start point or add advanced fit options
[fit_object, GoF, output] = fit(x, y, fit_type); 
% , 'StartPoint', [sx, sy]

% Unpack curve fit parameters
params = coeffvalues(fit_object);

% Compute the confidence interval for the fit parameters
param_CI = confint(fit_object, 0.95);

significance_check = (param_CI(1,:)<0 & param_CI(2,:)<0) | (param_CI(1,:)>0 & param_CI(2,:)>0);

% For a symmetric confidence interval, the uncertainty of the parameter 
% is half of the width of the confidence interval
param_uncertainty = 0.5*diff(param_CI);

conf_level = 0.95;  % 2.671 convention is 95% confidence

% New x variable over the domain
x_ = linspace(5, 55, 100);

% Construct prediction interval of the function
y_predint = predint(fit_object, x_, conf_level, 'functional', 'off');


% Plot results
% figure(1);
% name= figure;
% hold off;

% Do shading first so it appears on the bottom

% Define the upper bound of the shaded area
y_upper = y_predint(:, 2);

% Define the lower bound of the shaded area 
y_lower = y_predint(:, 1);

% Setup the arrays for shading a closed region
x2 = [x_, fliplr(x_)];
in_between = [y_upper; flipud(y_lower)];

% Shade the region
hf = fill(x2, in_between, 'm', 'DisplayName', '95% confidence bounds');
hold on;

% Adjust some of the parameters of the shading
hf.FaceAlpha = 0.3; % Make shading semi-transparent
hf.FaceColor = color; % Set color of shaded region
hf.EdgeColor = 'none'; % Set color of edge of shaded region

% Plot the least squares fit of the data
if min(significance_check>0)
    plot(x_, feval(fit_object, x_), '-', 'Color', color, 'DisplayName', 'LSQ fit'); 

else
    plot(x_, feval(fit_object, x_), '--', 'Color', color, 'DisplayName', 'Fit is Not Statistically Significant');

end

plot(x, y, 'o', 'Color', color, 'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'LineStyle','none', 'DisplayName', 'Data');  % Plot data last so it appears on top
% legend;
ImprovePlot;
