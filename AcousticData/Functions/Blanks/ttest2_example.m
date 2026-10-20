% Random data
rng(3); 
data_col_1 = randn(7,1)-0.2;
data_col_2 = randn(8,1);

plots_and_ttests(data_col_1, data_col_2, 0.95, {'Random data', 'Note: CIs overlap, and difference is not statistically significant'});

% Random data with overlapping CIs, but stat. sig. diff.
rng(3); 
data_col_1 = randn(7,1) + 1.0;
data_col_2 = randn(8,1);

plots_and_ttests(data_col_1, data_col_2, 0.95, {'Random data with offset', 'Note: CIs do not overlap'});

% Random data with overlapping CIs, but stat. sig. diff.
rng(3); 
data_col_1 = randn(7,1) - 0.01;
data_col_2 = randn(8,1);

plots_and_ttests(data_col_1, data_col_2, 0.95, {'Random data with offset', 'Note: CIs overlap, but difference might be statistically significant', 'See code comments for right-tail interpretation details'});


function [] = plots_and_ttests(data_col_1, data_col_2, conf, title_string)
    [m1,u1]=uncertainty(data_col_1);
    [m2,u2]=uncertainty(data_col_2);

    % Test decision for the null hypothesis that the data in vectors x and y 
    % comes from independent random samples from normal distributions with 
    % equal means and unequal and unknown variances

    % 2.671 convention: 95% confidence
    alpha = 1-conf; % 2.671 convention: p_crit = 0.05

    % Do samples have equal means?
    % If you don't know whether column 1 should or could be greater or less
    % than column 2, you should do a 2-tail analysis
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Most 2.671 projects use this formulation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Null hypothesis: 2 columns have same mean
    % Alternative hypothesis: 2 columns have means that are statistically significantly different
    % If p_2tail < p_crit, then you can reject the null hypothesis and
    % accept the alternative hypothesis
    tail = 'both';
    [~, p_2tail] = ttest2(data_col_1, data_col_2, 'Vartype','unequal', 'Alpha', alpha, 'Tail', tail);

    % If you know that group 2 should have a greater mean than group 1
    % Due to physical limits (e.g. time duration or scalar distance that 
    % cannot be negative) then you could use a left- or right- 1-tail metric
    % Null hypothesis: population mean of column 1 is less than population
    % mean of column 2
    % Alternative hypothesis: population mean of column 1 is greater than
    % population mean of column 2
    % If p_right < p_crit, then you can reject the null hypothesis and
    % accept the alternative hypothesis
    tail = 'right';
    [~, p_right] = ttest2(data_col_1, data_col_2, 'Vartype','unequal', 'Alpha', alpha, 'Tail', tail);

    % This is the complementary case of the right-tail.
    % Null hypothesis: population mean of column 2 is less than population
    % mean of column 1
    % Alternative hypothesis: population mean of column 2 is greater than
    % population mean of column 1
    % If p_left < p_crit, then you can reject the null hypothesis and
    % accept the alternative hypothesis
    tail = 'left';
    [~, p_left] = ttest2(data_col_1, data_col_2, 'Vartype','unequal', 'Alpha', alpha, 'Tail', tail);
   
    figure;
    subplot(1,2,1);
    hold on;
    h = scatter(1*ones(length(data_col_1),1), data_col_1, 'SizeData', 100);
    h.MarkerFaceColor = h.MarkerEdgeColor;
    h.MarkerFaceAlpha = 0.5;
    h.MarkerEdgeAlpha = 0.5;

    he = errorbar(1, m1, u1);
    he.LineStyle='none';
    
    hm = scatter(1, m1, 'SizeData', 300);
    hm.CData=h.CData;
    hm.MarkerFaceColor = hm.MarkerEdgeColor;
    he.Color=h.CData;

    h = scatter(2*ones(length(data_col_2),1), data_col_2, 'SizeData', 100);
    h.MarkerFaceColor = h.MarkerEdgeColor;
    h.MarkerFaceAlpha = 0.5;
    h.MarkerEdgeAlpha = 0.5;

    he = errorbar(2, m2, u2);
    he.LineStyle='none';
    he.Color=h.CData;
    
    hm = scatter(2, m2, 'SizeData', 300);
    hm.CData=h.CData;
    hm.MarkerFaceColor = hm.MarkerEdgeColor;

    xlabel('Data group');
    ylabel('Y Arb');
    title(title_string);
    xlim([0, 3]);
    xticks([1:2]);
    improvePlot;
    
    subplot(1,2,2);
    plot(NaN, NaN, '.');
    hold on;

    blank_line = '';
    line0 = 'At 95% confidence:';
    line1 = sprintf('Two tail: p_{m1==m2}=%.1e', round(p_2tail, 2, 'significant'));
    if p_2tail < alpha
        line2 = 'Can reject m1==m2; Can say m1, m2 are stat. sig. different'; 
    else
        line2 = 'Cannot reject m1==m2; Can say m1, m2 are not stat. sig. different';
    end
    line3 = sprintf('Right tail: p_{m1<m2}=%.1e', round(p_right, 2, 'significant'));
    
    if p_right < alpha
        line4 = 'Can reject m1<m2; Can say m1>=m2';
    else
        line4 = 'Cannot reject m1<m2';
    end
    line5 = sprintf('Left tail: p_{m1>m2}=%.1e', round(p_left, 2, 'significant'));
    
    if p_left < alpha
        line6 = 'Can reject m1>m2; Can say m1<=m2';
    else
        line6 = 'Cannot reject m1>m2.';
    end
    text_lines = {  line0, blank_line, ...
                    line1, line2, blank_line, blank_line, ...
                    line3, line4, blank_line, blank_line, ...
                    line5, line6};
    text(0,0.5,text_lines, 'FontSize', 20);
        axis off;
    set(gcf, 'Position',  [100, 100, 1500, 700])


end