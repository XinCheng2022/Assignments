clear all
% close all
clc
ptest= readtable('data_measurements.csv');
p= readtable('data_ground_truth.csv');

data = [p{:,4:5} p{:,10}  p{:,4} ]; % delta, theta1dot, v1r
dataTest = [ptest{:,3:5}  ptest{:,2}];
X = [data(:, 1:3) data(:, 1:3).^2 ];
[mm,nn] =size(X);
y = data(:, end);
m = length(y);
fprintf(' x = [%.5f %.5f %.5f %.5f %.5f %.5f], y = %.5f \n', [X(1:10,:) y(1:10,:)]');
% Scale features and set them to zero mean
[X, mu, sigma] = featureNormalize(X);

% Add intercept term to X
X = [ones(m, 1) X];

% Run gradient descent
% Choose some alpha value
alpha = 0.5;
num_iters = 4000;

% Init Theta and Run Gradient Descent 
theta = zeros(nn + 1, 1);
[theta, ~] = gradientDescentMulti(X, y, theta, alpha, num_iters)

% Display gradient descent's result
fprintf('Theta computed from gradient descent:\n%f\n%f\n%f',theta(1),theta(2),theta(3))

% Estimate the theta
for i = 1:m
    ThetaTEST (i)= theta(1)+...
        theta(2)*(dataTest(i, 1)-mu(1))/sigma(1)+...
        theta(3)*(dataTest(i, 2)-mu(2))/sigma(2)+...
        theta(4)*(dataTest(i, 3)-mu(3))/sigma(3)+...
        theta(5)*(dataTest(i, 1)^2-mu(4))/sigma(4)^2 +...
        theta(6)*(dataTest(i, 2)^2-mu(5))/sigma(5)^2 +...
        theta(7)*(dataTest(i, 3)^2-mu(6))/sigma(6)^2         ; % Enter your price formula here
   
    
    ThetaGT (i)= theta(1)+...
        theta(2)*(data(i, 1)-mu(1))/sigma(1)+...
        theta(3)*(data(i, 2)-mu(2))/sigma(2)+...
        theta(4)*(data(i, 3)-mu(3))/sigma(3)+...
        theta(5)*((data(i, 1)^2-mu(5))/sigma(4)) +...
        theta(6)*((data(i, 2)^2-mu(6))/sigma(5)) +...
        theta(7)*((data(i, 3)^2-mu(4))/sigma(6))    ;     
end 
figure
plot(ThetaTEST,'DisplayName','LR test')
hold on
plot(dataTest( :,4),'DisplayName','measurenet')
hold on
plot(data(:,end),'DisplayName','GT')
hold on
plot(ThetaGT,'DisplayName','LR GT')

legend


