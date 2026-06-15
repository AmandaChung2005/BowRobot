function [pks, x_locs] = find_N_peaks(x, y, N_peaks, doPlots)

    % Tries to find N peaks in the data y(x)
    % Input: x, y and N_peaks
    % Output:
    % pks: the amplitude of the peaks
    % x_locs: the x location of the peaks found

    % Make column vectors
    y = y(:);
    x = x(:);
    
    % First, plot the data
    if doPlots
        figure;
        plot(x, y)
        improvePlot;
    end
    
    thresh = 0.99*max(y);
    MAX_ITER = 50;
    for i=1:MAX_ITER
        [pks_sorted, locs_sorted, pk_widths, pk_proms] = findpeaks(y, x, ... 
                                    'SortStr', 'descend', ...       % Sort our peaks
                                    'MinPeakProminence', thresh);   % Minimum local height to be considered a peak
        
        N_pks_found = length(pks_sorted);
        if N_pks_found >= N_peaks
            break
        else
            thresh = 0.5*thresh;
        end
    end

    if N_pks_found < N_peaks
        fprintf('Warning: %d peaks requested, only %d peaks found...\n', N_peaks, N_pks_found);
        N_peaks = N_pks_found;
    end

    %%
    pks = pks_sorted(1:N_peaks);
    x_locs = locs_sorted(1:N_peaks);
    
    % Lastly, add the plotted peaks and label them
    if doPlots
        hold on;
        for i = 1:length(pks)
            peak_x_loc = x_locs(i);
            peak_val = pks(i);
            
            % Plot the marker on the peak
            h = plot(peak_x_loc, peak_val, 'rv'); 
            h.MarkerFaceColor = 'r';

            % Sometimes it may be necessary to tell MATLAB that the peak is
            % wider to make the peak integrals contain a wider domain. In
            % that case, increase WIDTH_ENHANCEMENT to 2.0 or more.
            WIDTH_ENHANCEMENT = 1.0;
            
            % Width of this peak
            width = WIDTH_ENHANCEMENT*pk_widths(i);
            
            % Local prominence of this peak
            prom = pk_proms(i);

            % Find the array index of the peak value
            idx_peak = find(y == peak_val, 1);
            
            % Now find the array indices of the start and end of the peak
            idx_num_int_start = find(x(1:idx_peak) < peak_x_loc - width/2.0, 1, 'last');
            idx_num_int_end = find(x(idx_peak:end) > peak_x_loc + width/2.0, 1, 'first') + idx_peak;
            
            % Make sure there is separation from start of peak, and peak
            if (idx_num_int_start == idx_peak)
                idx_num_int_start = idx_peak - 1;
            end
            
            % Make sure there is separation from end of peak, and peak
            if (idx_num_int_end == idx_peak)
                idx_num_int_end = idx_peak + 1;
            end
            
            % Domain over which to integrate the peak
            numIntDomain = idx_num_int_start:idx_num_int_end;
            
            % Numerical integration of the peak value
            NumIntOfPeak = trapz(x(numIntDomain), y(numIntDomain));
            
            % Define the upper bound of the fill area
            y_upper = y;

            % Define the lower bound of the fill area as y = 0
            y_lower = zeros(size(y));

            % Set a fill range, here for the numerical integration domain
            fill_range = false(size(y));
            fill_range(numIntDomain) = true;

            x1 = x(fill_range);
            x2 = [x1; flipud(x1)];
            inBetween = [y_upper(fill_range); flipud(y_lower(fill_range))];
            hf = fill(x2, inBetween, 'g');
            % Adjust some of the parameters of the fill
            hf.FaceAlpha = 0.5; % Make shading semi-transparent
            hf.FaceColor = 'red';
            hf.EdgeColor = 'red';
            
            label_str = sprintf('f=%.1f Hz\nInt = %.3e arb^2', x_locs(i), NumIntOfPeak);
            y_range = diff(get(gca, 'YLim')); % Want to separate labels from peaks
            y_offset = 0.03*y_range;
            text(x_locs(i), pks(i) + y_offset, label_str, 'HorizontalAlignment', 'center', 'FontSize', 16);
        end
    end
end
% If this script did not find the peaks you want, there are other ways you
% can guide the findpeaks function:

% 'MinPeakDistance', MIN_PEAK_DIST 
%       Use this if you know that peaks should have a minimum separation in x
%       e.g. You struck your drum at 1.0 Hz, so peaks should be separated by
%       at least 0.9 s
% 'MinPeakHeight', MIN_PEAK_HEIGHT
%       Use this if you only care about peaks of a certain minimum height
%       e.g. peak less than 0.5 V tall is noise, all real peaks are at
%       least 10 m, etc.
% 'Threshold', THRESHOLD
%       When you want at least THRESHOLD amplitude difference between a peak and its immediate neighbours.
%       Useful for excluding saturated peaks
% 'MinPeakProminence', MIN_PEAK_PROMINENCE
%       Use when you want peaks that drop off on both sides by at least MIN_PEAK_PROMINENCE before encountering a larger value.
%       Useful when several adjacent peaks get detected that you want to
%       count as one peak