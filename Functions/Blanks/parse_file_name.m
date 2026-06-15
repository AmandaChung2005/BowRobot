[input_file, input_path, success] = uigetfile('*.txt', 'Select input file');

if ~success
    fprintf('Error: no file selected...\n');
    return
end

% Assuming all filenames are of the form radius_X_length_Y.txt' and
% for the test file, it's radius_80_length_12.7.txt
% then we want to extract the radius is 80, and the length is 12.7:

file_name_contents = textscan(input_file, 'radius_%f_length_%f.txt');
[radius, length] = file_name_contents{:};

% The numbers we expect to be in the filename (i.e. 80, 12.7) are replaced
% by the format specified (%f), which tells MATLAB to look for a floating
% point number. The help entry for textscan gives more details on other 
% format string options if you wanted to extract other kinds of data encoded 
% in a file name.
