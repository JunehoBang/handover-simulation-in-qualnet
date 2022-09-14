function [value] = cdft(nu,x)
%CDFT Summary of this function goes here
%   Detailed explanation goes here
    firstTerm = 1/2;
    secondTerm =x*exp(gammaln((nu+1)/2)-gammaln(nu/2)); 
    hyperTerm = hypergeom([1/2, (nu+1)/2],3/2, (-1)*x^2/nu);
    denom = sqrt(pi*nu);    
    value = firstTerm +secondTerm*hyperTerm/denom;
end

