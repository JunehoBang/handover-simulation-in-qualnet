function [exp_mode, intervalProb] = crossingtimeprediction(mu, sig, alpha, beta, x, degree, tolerance)
%CROSSINGTIMEPREDICTION Summary of this function goes here
%   Detailed explanation goes here
    [N,M] = size(x);  %D: Dimension of original input
    Pi(1:M*degree+1)=1;
    for i=1:degree
        for j=i:degree
            Pi(j) = Pi(j)*x(1);  %squares of velocity
        end
    end
        
    for i=degree+1:degree*2
        for j=i:degree*2
            Pi(j) = Pi(j)*x(2); %squares of log-RSRP ratio
        end
    end
    location = mu'*Pi';
    shape = beta/alpha*(1+Pi*sig*Pi');
    nu = 2*alpha;
    exp_mode = exp(location);
    low = (log(exp_mode - tolerance)-location)/shape;
    high = (log(exp_mode + tolerance)-location)/shape;
    intervalProb = tcdf(high, nu) - tcdf(low, nu);
 end

