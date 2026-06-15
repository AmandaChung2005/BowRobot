%% Make up some data to play with
% You could replace this section of code to import your own experimental
% t,y data
t=0:0.001:1.0; t=t(:);
f1 = 6;
f2 = 12;
% Start with some noise
y = 0.1*randn(length(t),1);
range1 = t < 0.5;
y(range1) = y(range1) + 5.0*sin(2*pi*f1*t(range1));
y(~range1) = y(~range1) + 5.0*sin(2*pi*f2*t(~range1));

% Clip low values to create impulse train
y(y < 4.5) = 0.0;

%% Pre-processing
dt = mean(diff(t)); % Mean time between samples
fs = 1.0/dt;        % This is the mean sample frequency in Hz

%% Code to identify segments based on peaks
t_sep = 0.05;           % Minimum time separation between peaks
N_sep = ceil(t_sep*fs); % Convert time separation to array entries
[peaks, peak_indices] = findpeaks(y, 'MinPeakProminence', 0.05, 'MinPeakDistance', N_sep);

figure;
plot(t, y); hold on;
xlabel('Time (s)');
ylabel('Amplitude (V)');

% plot(t(peak_indices), peaks, 'r+');
% title('Raw data with peaks');
title('Raw data');
improvePlot;
changePlotSize(1200, 600);


%% Code to separate each segment based on the peak locations
t_peak_pad = 0.05;              % Amount of time a segment "starts" before its peak
N_pp = ceil(t_peak_pad * fs);   % Padding time converted into number of array points

N_segments = (length(peak_indices)-1);
seg_starts = zeros(N_segments,1);
seg_ends = zeros(N_segments,1);

% Skip the first peak
for i=1:N_segments
    st = max(peak_indices(i) - N_pp, 1);
    en = min(peak_indices(i+1) - N_pp, length(t)-1);
    
    seg_starts(i) = st;
    seg_ends(i) = en;
end

% % Include the last segment
% hit_starts = [hit_starts; hit_ends(end)];
% hit_ends = [hit_ends; length(t)];
% N_strikes = N_strikes + 1;
% 
%% Code for multi-coloured graph of segments (separate and superimposed)
 
figure;

for i=1:N_segments
    this_segment = seg_starts(i):seg_ends(i);

    subplot(2,1,1); hold on;
    plot(t(this_segment), y(this_segment));    
    if (i == 1)
        xlabel('Time (s)');
        ylabel('Amplitude (V)');
        title('Separated strikes');
    end
    
    subplot(2,1,2); hold on;
    plot(t(this_segment)-t(this_segment(1)), y(this_segment));
    if (i == 1)
        xlabel('Time (s)');
        ylabel('Amplitude (V)');
        title('Superimposed strikes');
    end
      
end

% Clean up each figure
box on; improvePlot; 
    
function [] = changePlotSize(plot_w__px, plot_h__px)
    set(gcf, 'rend', 'painters', 'pos', [100 100 plot_w__px plot_h__px]);
end