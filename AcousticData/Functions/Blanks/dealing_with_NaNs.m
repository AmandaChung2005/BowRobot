% Dealing with NaNs
x=0:2:75;
data = 2.0*sin(0.1*x);
data(1:3:length(x)) = NaN;

figure;
% subplot(2,1,1);
plot(x, data, 'ko');
title('Data with NaNs');
improvePlot;
NaN_mask = isnan(data);
data = fillmissing(data, 'linear');

% subplot(2,1,2)
figure;
plot(x, data, 'ko'); hold on;
h = plot(x(NaN_mask), data(NaN_mask), 'mo');
title({'Replace NaN with', 'linear interpolation'});
improvePlot;
h.MarkerSize = 5;