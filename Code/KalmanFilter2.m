%% Parameters to set

% sampling frequency
samplingFreq = 2; %kilohertz -> sample 1,000 times per second

% stop time (decide when to end the simulation)
stopTime = 1; %seconds

% wave frequency
freq = 60; %hertz

% wave amplitude
amplitude = 1; %volts % TODO do I need to calculate the A* and Ao?  Unsure what they are
% VERIFIED amplitude is okay, all it does is change the height (duh)
% doesn't change anything about the freq of the sine wave
% SO probably not a huge deal

% phase
phase = 1; % TODO they didn't provide this in the experimental setup...just assume it is 1? 0?

% random noise attack?
randNoiseAttack = true;

%% Calculate timesteps

% convert freq to use seconds
samplingFreq = samplingFreq*1000;

% timestep size
dt = 1/samplingFreq;

% generate all timesteps for the sim aka when each sampling event will
% occur
t = (0:dt:stopTime);

%% Generate measurements

% generate a randomized measurement for each timestep
y = zeros(1,size(t,2));
for idx = 1:size(t,2)
    timeNow = t(idx);
    C = [cos(2*pi*freq*timeNow) -sin(2*pi*freq*timeNow)];
    x1 = amplitude*cos(phase); 
    x2 = amplitude*sin(phase);
%     x1 = 1;
%     x2 = 1;
    x = [x1 ; x2];
    lower = -1;
    upper = 1;
    randomNum = lower+(upper-lower)*rand(1,1);
    v = randomNum; % TODO this should be Gaussian, but just make it a random num for now
%     v = 0;
    y(idx) = C*x + v;
end
figure()
plot(t,y)
xlabel('time')
ylabel('voltage signal (with noise)')
title('Sensor measurements of voltage, including Gaussian white noise')


%% Initialize state

% these nums are given in the paper
x1 = 0;
x2 = 0;
x = [x1 ; x2];
P = eye(2);

%% Time loop

% TODO what is this called
A = eye(2); % defined in paper to always be this value

% process noise covariance matrix
Q = zeros(2,2); % TODO set values for this

% measurement noise covariance matrix
R = 1; % TODO set values for this

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

x_ = [];
for idx = 1:size(t,2)
    timeNow = t(idx);
    % time update
    x = A*x
    x_ = [x_;x];
    P = A*P*A' + Q
    
    % measurement update
    C = [cos(2*pi*freq*timeNow) -sin(2*pi*freq*timeNow)];
    K = P * C' * inv(C*P*C' + R);
    
    x = x + K*(y(idx)+randomNums(idx) - C*x);
    P = P - K*C*P;  
end


%% Analysis
x1_kalman = x_(1:2:end);
x2_kalman = x_(2:2:end);
voltage_kalman = zeros(1,size(t,2));

for idx = 1:size(t,2)
    timeNow = t(idx);
    voltage = x1_kalman(idx)*cos(2*pi*freq*timeNow) - x2_kalman(idx)*sin(2*pi*freq*timeNow);
    voltage_kalman(idx) = voltage;
end

gcf
hold on
plot(t,voltage_kalman,'r')

hold on
lowertime = 1001;
uppertime = 1500;
plot(t(lowertime:uppertime),y(lowertime:uppertime)'+randomNums(lowertime:uppertime), 'm')
plot(t,y+randomNums','m')
legend('signal','kalman estimate')
