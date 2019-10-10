close all
clear
clc

%% Define motor parameters
J = 0.0001;
b = 0.001;
K = 0.14; % MMGM
R = 7; % MMGM
L = 0.1;
s = tf('s');
motor_tf = K / ( s*( L*J*s^2 + (L*b+R*J)*s + (R*b+K^2) ) )

%% Simulate open loop system with step response
linearSystemAnalyzer('step', motor_tf, 0:0.1:5)

%% Check ctrb and obsv
close all
clc

A = [0 1 0
    0 -b/J K/J
    0 -K/L -R/L];

B = [0
    0
    1/L];

C = [1 0 0];

rank(ctrb(A, B))
rank(obsv(A, C))

%% Add control
close all
clc

Kp = 2.5;
Ki = 0;
Kd = 1;

C = pid(Kp, Ki, Kd)
cl_sys = feedback(C*motor_tf, 1)

t = 0:0.01:2;
opt = stepDataOptions;
opt.StepAmplitude = 2*pi;
step(cl_sys, t, opt)
grid
title('Step Response with Control')

%%
close all