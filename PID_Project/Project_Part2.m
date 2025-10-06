%Case 1

s=tf('s');
Gp=0.018/((s+0.06)*(s+0.1)*(s+3))
Gc = 1;
G = Gp*Gc;
figure(1)       %Open Loop System
step(G)
title("Step Response with No Controller")

figure(2)       %Unity Feedback System
T=feedback(G,1);
step(T)
title("Step Response with Unity Feedback system")


% The above transfer function plots the step response of the system with plant and no controller with
% the system. The Step Response of the system improved upon the incorporation of Unity Feedback
% system. From the step response of “Close Loop TF with Unity Feedback System”, the settling
% time as well as overshoot of the system was much improved with the unity feedback system.


% K=0.01

% Gc=((s+0.3)*(s+1.5))/s
% Hk=feedback(Gp*Gc,1)
% figure(3)
% step(Hk)
% G=Gp*Gc

%%
%Case2a

% Here gain value of Max Overshoot of 2% and settling time of 9 seconds, was obtained using both
% the Routh stability Criterion and the Root Locus Method.

Mos = 0.02;
zeta = -log(Mos)/sqrt(pi^2+(log(Mos)^2))
theta = acosd(zeta)
ts = 9;
zeta_wn = 4/ts
wd = zeta_wn*tand(theta) 
wn = 4/(ts*zeta);

s = tf('s');
Gp = 0.018/((s+0.06)*(s+0.1)*(s+3));
k = 5.71;
Td = 59;
Gc = Td*s*(k*(s+0.3)*(s+1.5))/s;
G = Gp*Gc;
T = feedback(G,1)
step(T)
stepinfo(T)

rlocus(G)
sgrid(zeta,0)
[k, CLpoles] = rlocfind(G)
sisotool(G)

% Using PD controller, the step response of the given was obtained. Sisotool was used to adjust the gain value that satisfies the
% system performance parameters of Max Overshoot of 4% and Settling time of 9 seconds.

%%
% %Case 2b. 
% Using root locus method for Max Overshoot of 4% and Settling time of 9 seconds, the design point
% as well as value of gain, k, was calculated. PID controller was utilized to achieve the required
% performance parameters of the system.
s = tf('s');
k = 5.2;
Td = 72;
Gc = (k*(s+.3)*(s+1.5))/s; %Controller
Gp = .018/((s+.06)*(s+.1)*(s+3)); %Plant
G = Gp*Gc;
H = feedback(G*(Td*s), 1);
step(H)
stepinfo(H)
sisotool(H)
% The calculated value of gain, k, was adjusted using the sisotool to achieve the required system
% performance parameters. This adjusted gain value was used to make sure that the system had the
% steady state error of zero.
t = 0:0.1:50;
u = t;
[y,t,x] = lsim(T,u,t);
plot(t,y,'k',t,u,'m')
xlabel('Time (sec)')
ylabel('Amplitude')
title('Steady State Error - Input-purple, Output-black')

% using SIMULINK the unity negative disturbance rejection was checked. The Simulink model
% and graph are given below. This satisfied the requirements for overshot, settling time and steady
% state error of zero. The PID Controller was tuned using linearization plant