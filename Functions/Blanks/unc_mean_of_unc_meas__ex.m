% CASE 1A
x = [5.1, 5.0, 4.9, 4.9, 4.9];
x = 5*[1,1,1,1,1];
u_x = [0.1, 0.1, 0.1, .1, .1];
conf = 0.95;

mean_x = mean(x);
std_x = std(x);
N = length(x);

dof = N-1; % Appropriate here because this is estimate of mean
t_factor = tinv(1.-(1.-conf)/2., dof); 

U_AVG = t_factor*std_x/sqrt(N);
U_INDIV = sqrt(sumsqr(u_x./N));

U_TOT = sqrt(U_AVG.^2 + U_INDIV.^2);

