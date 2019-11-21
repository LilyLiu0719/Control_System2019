%% Setup the plant
s = tf('s');
P = -1.202*(s-1)/(s*(s+9)*(s^2+12*s+56.25));
% figure; 
% subplot(2,1,1);
% step(P,2);
% title('Open-loop step response');

%% Design the PID controller  %J = 0.0557
Kp = 500;
Ki = 0;
Kd = 0;
C = -(s+9)*(s^2+12*s+56.25)/(s-1);
C1 = pid(Kp,Ki,Kd);
H = feedback(C*C1*P,1);
S = stepinfo(H);
ts = S.SettlingTime;
tr = S.RiseTime;
Mo = S.Overshoot;

%% Simulate the closed-loop step response
figure; 
subplot(2,1,2);
step(H,10);
title('Closed-loop step response');
%J = 10*tr + ts + 20*Mo;

%% Calculate the steady state error
[y,t] = step(H);
ess = abs(1-y(end));
J = 10*tr + ts + 20*Mo + 100*ess;
