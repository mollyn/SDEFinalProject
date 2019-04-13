%% Initialize
% initial state value
x = [0 ; 0]; % as stated in experimental setup on pg 375
% initial
P = zeros(2,2); % as stated in experimental setup on pg 375

Q = []; % process noise matrix

% sampling frequency
samplingFreq = 2; % kilohertz -> sample 1,000 times per second
samplingFreq = samplingFreq * 1000;
dt = 1/samplingFreq;

stopTime = 1; % stop after 10 seconds

t = (0:dt:stopTime); % these are the timesteps to use, when each sample will occur

% frequency of sine wave (as stated in experiemtnal setup)
freq = 60; % hertz

% sine wave signal
sinusoidalSignal = sin(2*pi*freq*t);

plot(t,sinusoidalSignal)
shg

%% Create measurement stream

measurementStream = [];
C = [cos(omega*t) -sin(omega*t)];
v = []; %measurement noise
y = C*measurementStream + v;

R = []; % measurement noise covariance matrix

%% Time update

A = eye(2);
x = A*x;

P = A*P*A' + Q;

%% Measurement update
K = P*C'*inv(C*P*C' + R);
P = P - K*C*P;
x = x + K*(y-C*x);



%% MEASUREMENTS DONE RIGHT

% sampling frequency
samplingFreq = 2; % kilohertz -> sample 1,000 times per second
samplingFreq = samplingFreq * 1000;
dt = 1/samplingFreq;
stopTime = 1; % stop after 10 seconds
t = (0:dt:stopTime); % these are the timesteps to use, when each sample will occur

% frequency of sine wave (as stated in experiemtnal setup)
freq = 60; % hertz
amplitude = 1; % volt % TODO what are A*? otherwise the x's are going to be all the same
phase = 1; % TODO where does this come from

y = zeros(size(t,2));
for idx = 1:size(t,2)
    time = t(idx);
    C = [cos(2*pi*freq*time) -sin(2*pi*freq*time)];
    x1 = amplitude*cos(phase); % VERIFIED amplitude is okay, all it does is change the height (duh) doesn't change anything about the freq of the sine wave
    x2 = amplitude*sin(phase);
%     x1 = 1;
%     x2 = 1;
    x = [x1 ; x2];
    v = rand([1,1]); % TODO this should be Gaussian, but just make it a random num for now
    y(idx) = C*x + v;
end
figure()
plot(t,y)

