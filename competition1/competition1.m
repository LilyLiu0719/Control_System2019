%% Setup the plant
s = tf('s');
P = -1.202*(s-1)/(s*(s+9)*(s^2+12*s+56.25));
% figure; 
% subplot(2,1,1);
% step(P,2);
% title('Open-loop step response');

%% Design the PID controller  %J = 27.5137
Kp = 220.6;
Ki = 2.3;
Kd = 78;
C = pid(Kp,Ki,Kd);
% C = -(s*(s+9)*(s^2+12*s+56.25))/(1.202*(s-1)*(s^2+100*s+10000));
H = feedback(C*P,1);
S = stepinfo(H);
ts = S.SettlingTime;
tr = S.RiseTime;
Mo = S.SettlingMax;

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
