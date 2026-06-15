clear all; close all; clc;
% Let's make some data with which to play
rng default; % For repeatability
mu = 68.1;
sig = 6.9;
N_pts = 216;
y = normrnd(mu, sig, N_pts, 1);

%%

N_bins = 20;

figure; hold on;
h = histogram(y, N_bins, 'Normalization', 'pdf', 'DisplayStyle', 'Stairs', 'LineWidth', 2, 'DisplayName', 'Histogram');
xlabel('Amplitude');
ylabel('Probability');
h.EdgeColor = 'red';
bin_edges = h.BinEdges;

% Fit data to a Guassian model (i.e. normal PDF) 
sd = std(y);
mu = mean(y);
x = linspace(min(bin_edges), max(bin_edges), 100);
g = normpdf(x, mu, sd);
legend_str = sprintf('Norm. pdf (mu=%.1f, sigma=%.1f)',mu,sd);
plot(x, g, '--', 'LineWidth', h.LineWidth, 'Color', h.EdgeColor, 'DisplayName', legend_str);
legend('show');

improvePlot;
