%file meansd.m
x = [1,2,3,4,5];

mu = mean(x);
s_s1 = std(x);
s_s2 = std(x,0);

% both definitions above give sample standard deviation 

s_p = std(x,1);

% This gives population standard deviation

mu
s_s1
s_s2
s_p
