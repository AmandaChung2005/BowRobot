clc;        % Clears the command window
clear all;  % Clears variables form the workspace
close all;  % Closes all open figures

% Select your CSV data file
[fn, pn] = uigetfile('*.csv');

% Read in data
T = readtable([pn, fn]);

% Rename columns for later reference
T.Properties.VariableNames = {'Time' 'Position' 'Triceps_potential' 'Chest_potential'};

% Print out the first 3 rows of all columns 
T(1:3,:)
%%

% I'm not sure how you want to identify start/end of each repetition:

% Choose a variable you want to treat (you could also extend this to loop 
% over all variables)
% Option 1, position
var_to_plot = T.Position;
var_name = 'Position (m?)';

var_to_plot = T.Triceps_potential;
var_name = 'Triceps potential (V)';


% Your data has some empty cells because  different channels were
% sampled at different rates. Empty cells were read into the table as NaN
% We will make a mask to identify rows that were empty in original CSV file
% Then we will exclude those rows...

figure;
nan_mask = isnan(var_to_plot); 
plot(T.Time(~nan_mask), var_to_plot(~nan_mask)); hold on;
xlabel('Time (s)');
ylabel(var_name);
title('Select boundary points. Press return when done.');
improvePlot();

% This will let you select points on the graph that mark the start and end
% of each repetition. When you are done selecting points, press Enter
% (Return)

[x,y,button] = ginput; % This captures x and y coords, and what button was pressed
if (length(x) < 2)
    fprintf('Error. Need at least 2 points.\n');
end

% Sort the points in ascending order of time
[x, indices] = sort(x);
y = y(indices); % We don't really care about the y-coord, but sort it anyway

%%
% Add a circle at the click points
plot(x,y,'ko','MarkerSize',20,'MarkerFaceColor','y');

N_plt = length(x)-1;
nc = ceil(N_plt/2);

% Plot the individual repetitions. You could replace the plot routine
% with some other operation, e.g. finding max or mean value, integrating,
% etc.

figure;

for i=1:N_plt
    % Find the start and end of the repetition
    ist = find(T.Time >= x(i), 1);
    ien = find(T.Time >= x(i+1), 1);
    
    range_mask = ist:ien;
    
    subplot(2, nc, i);
    t = T.Time(range_mask);
    v2p = var_to_plot(range_mask);
    nan_mask = isnan(v2p); % Want to ignore NaN values if applicable
    
    plot(t(~nan_mask), v2p(~nan_mask)); hold on;
    title(sprintf('Repetition %d',i));
    xlabel('Time (s)');
    ylabel(var_name);
end
