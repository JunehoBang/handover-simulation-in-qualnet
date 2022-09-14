function [mu, sig, alpha, beta] = trainMybrm(degree)
%TRAINMYBRM Summary of this function goes here
%   Detailed explanation goes here

% Write mk and sigmak, k=1,...,K line by line in the output file
% Each sigma k entry is on a line
    fileId = fopen('training.txt','r');
    formatSpec = '%lf %lf %lf';
    if fileId > 0
        sizeX  = [3 Inf];
        X=fscanf(fileId, formatSpec, sizeX);
        fclose(fileId);
        [D,N] = size(X);  %D: Dimension of original input
        D=D-1;            %N: Number of training datum
        PI(1:D*degree+1,1:N)=1;
        for i=1:degree
            for j=i:degree
                PI(j,:) = PI(j,:).*X(1,:);  %squares of velocity
            end
        end
        
        for i=degree+1:degree*2
            for j=i:degree*2
                PI(j,:) = PI(j,:).*X(2,:); %squares of log-RSRP ratio
            end
        end
        temp=PI';
        PI=temp;
        [N,M]=size(PI); %M: dimension of transformed input
        y = X(3,:);
        y=y';
    else
        
        
    end
    
    PIV = sym(PI'*PI);
    sig = double(inv(vpa(PIV)));
    mu = sig*PI'*y;
    nu=N-M;
    tau_s= (y-PI*mu)'*(y-PI*mu)/nu;
    alpha = nu/2;
    beta = nu*tau_s/2;
end

