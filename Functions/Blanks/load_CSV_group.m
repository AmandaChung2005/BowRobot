%% Code to select multiple input data files, and iterate over them
clc;
close all;

FILE_EXT = 'csv'; % Select your file extension (e.g. csv, txt, wav, etc.)

[all_data_file_names, data_file_path, success] = uigetfile(['*.',FILE_EXT], 'Select input file(s)', 'MultiSelect', 'On');

if ~success
    fprintf('Error: No files selected...\n');
    return
end

% Handle case where only one file is selected
if ischar(all_data_file_names)
    all_data_file_names = {all_data_file_names};
end
    
N_files = length(all_data_file_names);
for ii = 1:N_files
    this_file_name = all_data_file_names{ii};
    fprintf('Opening %s ...\n', this_file_name);   
    full_file = fullfile(data_file_path, this_file_name);         
    
    % Your code to run for each file here...
    % Example:
    % Load the CSV file
    % [headers, data] = load_CSV_file(full_file);
    % Assign columns to variables
    % var1 = data(:,1);
    % var2 = data(:,2);
end