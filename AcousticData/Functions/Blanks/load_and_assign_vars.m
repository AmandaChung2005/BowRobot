[headers, data] = load_CSV_file;
%%
% By manual inspection of the CSV, we know that we expect force, 
% acceleration and time columns, so we can auto-assign columns to variables
% Sometimes LoggerPro re-organizes columns, this is a nice way to insulate
% against the file configuration details

for i=1:length(headers)
    header = headers{i};
    
    if contains(header, 'accel', 'IgnoreCase', true)
        accel = data(:, i);
        
    elseif contains(header, 'time', 'IgnoreCase', true)
        time = data(:, i);
        
    elseif contains(header, 'force', 'IgnoreCase', true)
        force = data(:, i);
        
    end
    
end