%% create data
a = 1;
b = 2;
c = 1;
N = 100; % number of simulation points
w_sigma = 1;

x_data = [0:N-1]/N;
y_data = exp(a*x_data.^2 + b*x_data + c) + randn(1,N);
%% Gaussian-Newton method
iterations = 1e2;
residue = zeros(1,N);
cost = 0;
lastCost = 0;

est = [2,-1,5]';
dest = [0,0,0]';
H = zeros(3,3);
g = [0,0,0]';
x_data_abs2 = (x_data.^2);
Jacob = zeros(3,N);

tic;
for i = 1:iterations    
    fx = exp(est(1)*x_data_abs2+est(2)*x_data + est(3));
    residue = y_data - fx;
    cost = sum(residue.^2);
    Jacob(1,:) = -x_data_abs2.*fx;
    Jacob(2,:) = -x_data.*fx;
    Jacob(3,:) = -fx;

    H = Jacob*Jacob';
    g = sum(-Jacob.*residue,2);    
    dest = H\g;
    if(isnan(dest(1)))
        break;
    end
    
    if(i>1 && cost>=lastCost)
        break;
    end
    
    est = est + dest;
    lastCost = cost;

end
toc;
