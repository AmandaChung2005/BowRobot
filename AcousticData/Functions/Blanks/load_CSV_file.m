function [headers, data] = load_CSV_file(input_file)
% function [headers, data] = load_CSV_file(input_file)
% input_file is optional argument
% if no file is specified, UI prompt will open

    switch nargin
        case 0
            [fn, pn] = uigetfile('*.csv');
            filename = strcat(pn, fn);
            
        otherwise
            filename = input_file;
    end


% Read the column headers in the first line of the CSV file
fid = fopen(filename);
headers_ = fgetl(fid); % Will read as one line
fclose(fid);

% Separate the line into individual column headers based on comma separator
headers = strsplit(headers_, ',');

% Read CSV file as data, but ignore the header
data=csvread(filename, 1, 0); 

[~, name, ~] = fileparts(filename);
fprintf('Finished loading file %s...\n', strrep(name, '.csv',''));

end

