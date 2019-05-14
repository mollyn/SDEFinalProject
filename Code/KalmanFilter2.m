%% Add utils to path
% NOTE: you must cd to where this file is saved on the system, and ensure
% that createColors is stored at the same file level
run('createColors.m')

%% Parameters to set (given on pg 375 of paper)

% sampling frequency
samplingFreq = 1; %kilohertz -> sample 1,000 times per second

% stop time (decide when to end the simulation)
% for running locally on laptop, can't be too large
stopTime = 20; %seconds

% wave frequency
freq = 60; %hertz

% wave amplitude
amplitude = 1; %volts

% phase
phase = 1; %the only quantity not given in the experimental setup
           % I chose 1 for simplicity

% whether to execute attack or not
randNoiseAttack = true;
randNoiseAttack = false;


%% Calculate timesteps

% convert freq to use samples per second
samplingFreq = samplingFreq*1000;

% timestep size
dt = 1/samplingFreq;

% generate all timesteps for the sim 
% (a sample is taken at each timestep)
t = 0:dt:stopTime;


%% Generate measurements

% generate a randomized measurement for each timestep in the sim
y = zeros(1,size(t,2)); % measurements with noise
yNoNoise = zeros(1,size(t,2)); % measurements without noise
for idx = 1:size(t,2)
    timeNow = t(idx);
    % calculate quantities as described in the paper
    C = [cos(2*pi*freq*timeNow) -sin(2*pi*freq*timeNow)];
    x1 = amplitude*cos(phase); 
    x2 = amplitude*sin(phase);
    x = [x1 ; x2];
    % generate measurement noise
    % (Gaussian with mean=0, variance=1)
    v = normrnd(0,1);
    y(idx) = C*x + v;
    yNoNoise(idx) = C*x;
end
ax1 = subplot(1,2,1);
plot(t,y,'Color',orange)
xlabel('time')
ylabel('voltage signal (with noise)')
title('Sensor measurements of voltage, including Gaussian white noise')

ax2 = subplot(1,2,2);
plot(t,yNoNoise,'Color',orange)
xlabel('time')
ylabel('voltage signal (no noise)')
title('Sensor measurements of voltage, no measurement noise')

linkaxes([ax1 ax2], 'xy')


%% Initialize state before using Kalman Filter

% These are given on pg 375 of paper
x1 = 0;
x2 = 0;
x = [x1 ; x2];
P = eye(2);


%% Time loop

A = eye(2); %defined in the paper

% process noise covariance matrix
Q = [.01 .01 ; .01 .01];

% measurement noise covariance
R = 1;

% create the attack
if randNoiseAttack
    lower = -10;
    upper = 10;
    randomNums = lower+(upper-lower)*rand(size(t,2),1);
    beginAttack = 500;
    endAttack = 1501;
    randomNums(1:beginAttack-1) = 0;
%     randomNums(1001:1500) = keep
    randomNums(endAttack:end) = 0;
else
    randomNums = zeros(size(t,2),1);
end

% Kalman loop
x_ = []; %this variable will keep track of the state at each time step
for idx = 1:size(t,2)
    timeNow = t(idx);
    % time update
    x = A*x; %eqn 15
    x_ = [x_;x]; %save off the state
    P = A*P*A' + Q; %eqn 16
    
    % measurement update
    C = [cos(2*pi*freq*timeNow) -sin(2*pi*freq*timeNow)]; %calculate C
    K = P * C' * inv(C*P*C' + R); %eqn 17
    
    x = x + K*(y(idx)+randomNums(idx) - C*x); %eqn 19
    P = P - K*C*P; %eqn 18
end


%% Analysis
x1_kalman = x_(1:2:end);
x2_kalman = x_(2:2:end);
voltage_kalman = zeros(1,size(t,2));

% Need to calculate the voltage values based on the Kalman filter estimates
for idx = 1:size(t,2)
    timeNow = t(idx);
    voltage = x1_kalman(idx)*cos(2*pi*freq*timeNow) - x2_kalman(idx)*sin(2*pi*freq*timeNow);
    voltage_kalman(idx) = voltage;
end

figure()
plot(t,voltage_kalman,'Color',purple,'linewidth',2)
hold on
if randNoiseAttack
    plot(t,y+randomNums','Color',green)
else
    plot(t,y,'Color',green)
end
legend('kalman estimate', 'signal')
xlabel('time')
ylabel('voltage')
title('Voltage signal with Kalman estimates')
