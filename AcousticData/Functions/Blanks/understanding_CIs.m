close all;
clear all;
clc;
%%understanding confidence intervals
rng default; % For repeatability
% Createa  population of 1000000 random numbers from a normal distribution
% with mu = 5.0, and standard deviation = 2.0
population_size = 1000000;
popn = normrnd(5.0, 2.0, 1, population_size);

mu_popn = mean(popn); % Confirm that the simulated population has the expected mean

% Visualize the distribution of the simulated population with histogram
figure;
histogram(popn, 30, 'Normalization','pdf')
xlabel('Value');
ylabel('Probability');
title('Simulated population');
improvePlot;

% Randomly draw SAMPLE_SIZE elements from popn. Each element will be a
% different drawing because we draw without replacement.
% From each sample of SAMPLE_SIZE elements, 
% We will compute the mean, standard deviation and confidence interval
% Then we will see if the true population mean (which we know this time) is
% inside the confidence interval, or whether it's not.
% We will repeat this procedure i times, and if i=1000, we expect that we
% our 95% confidence interval would contain the true population mean (mu_p)
% 950 times.

SAMPLE_SIZE = 40;
CONFIDENCE = 0.95;

for iter_max = [10, 100, 1000, 10000]
    ctr_CI_valid = 0;
    for i = 1:iter_max
        % Sample from popn without replacement
        sample = datasample(popn, SAMPLE_SIZE, 'Replace', false);

        sample_sd = std(sample);
        t_factor = tinv((1+CONFIDENCE)/2., SAMPLE_SIZE - 1);
        mu_bar = mean(sample);
        u_mubar = t_factor*sample_sd/sqrt(SAMPLE_SIZE);
        CI_lower_bound = mu_bar - u_mubar;
        CI_upper_bound = mu_bar + u_mubar;

        if (mu_popn > CI_lower_bound) && (mu_popn < CI_upper_bound)
            ctr_CI_valid = ctr_CI_valid +  1;
        end

        if i==1 && false
        fprintf('\n');
        fprintf('Pop. mean = %.3f, Pop. sd = %.3f', mean(popn), std(popn))
        fprintf('\n')
        fprintf('Sample mean = %.3f, Sample sd = %.3f', mu_bar, sample_sd)
        fprintf('\n')
        fprintf('Sample 0.95 CI = (%.3f,%.3f)', CI_lower_bound, CI_upper_bound)
        fprintf('\n');
        end
    end

    fprintf('In %6d trials, CI was valid %d times\n', i, ctr_CI_valid);

end

CONFIDENCE = 0.95;
c_str = sprintf('%i%% CI',100*CONFIDENCE);
sample_sizes = [2, 3, 4, 5, 6, 10, 20];

% Sample without replacement
for k = 1:100
   hfig = figure('rend','painters','pos',[10 10 800 800]);
   set(hfig, 'Visible', 'off');
   overall_sample = datasample(popn, max(SAMPLE_SIZE), 'Replace', false);
   for SAMPLE_SIZE = sample_sizes
        hold on;
        ctr_CI_valid = 0;
        sample = overall_sample(1:SAMPLE_SIZE);

        sample_sd = std(sample);
        nu = SAMPLE_SIZE - 1;
        t_factor = tinv((1+CONFIDENCE)/2., nu);
        mu_bar = mean(sample);

        u_mubar = t_factor*sample_sd/sqrt(SAMPLE_SIZE);

        CI_lower_bound = mu_bar - u_mubar;
        CI_upper_bound = mu_bar + u_mubar;
        plot([0 25], mu_popn*[1 1], 'g-', 'LineWidth', 5);
        plot(SAMPLE_SIZE, mu_bar, 'ko' , 'MarkerSize', 10, 'LineWidth', 3);
        plot(SAMPLE_SIZE, CI_upper_bound, 'rv' , 'MarkerSize', 10, 'LineWidth', 3);
        plot(SAMPLE_SIZE, CI_lower_bound, 'r^' , 'MarkerSize', 10, 'LineWidth', 3);
        lh=plot(SAMPLE_SIZE*[1,1], [CI_upper_bound, CI_lower_bound], 'r-', 'LineWidth', 2);
        lh.Color=[1,0,0,0.5]; % Transparency hack
        legend('True pop. mean', 'Sample est. mean', [c_str,  ' Upper Limit'], [c_str, ' Lower Limit']);
        xlabel('Number of samples');
        ylabel('Value');
        set(gca,'FontSize',20);
        box on;
        axis([0, 22, -15, 25]);
        xticks([0:1:22]);
        grid on;
        hold off;
   end
   if (CI_lower_bound > mu_popn) || (CI_upper_bound < mu_popn)
        print(hfig, ['Outlier_iter_' num2str(k)], '-dpng')
   end
   M(k) = getframe(hfig);
   close(hfig);
end

%%
% close all;
figure('rend','painters','pos',[10 10 800 800]);
movie(gcf, M, 1, 20)
