%% Toy example link : https://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
clear
clc

%% Initialization
Ts = 0.1;  % Sampling Time
time=0:Ts:100;
N = length(time)-1;

% For Plant
x0 = [0;0];    % Position, Velocity
x(:,1) = x0;
a = normpdf(time,50,20); % Acceleration Profile

% For Kalman Filter
sigma_a = 0.1;
mu=0;
x_hat(:,1) = x0;
P_k{1} = 100*eye(2,2);
P_k_norm(1) = norm(P_k{1});

% Velocity Profile
Velocity(1:20) = 0; % m/s
Velocity(20:100) = 0.5;
Velocity(100:300) = 1.2;
Velocity(300:500) = 1;
Velocity(500:700) = 0;
Velocity(700:800) = 0.7;
Velocity(800:1000) = 0;

%% Plant Model
F = [1 Ts; 0 1];
G = [Ts^2/2; Ts];
H = [1 0];         % we have only position's measurement 

% Check System Controllability
if(rank(ctrb(F,G))==2)
   disp('System is controllable !') 
end
% Check System Observability
if(rank(obsv(F,H))==2)
   disp('System is observable !') 
end

%% PID Controller
Kp = 0.33; %7.49220278328499
Ki = 0.01;

%% Velocity Estimate using Kalman Filter
Q = sigma_a^2;
R = 0.1;
noise = sigma_a*randn(1,N)+mu; % Generate White Noise
noise(1:20)=0;

Error = zeros(1,length(N));
Prop = zeros(1,length(N));
Integral = zeros(1,length(N));

for t=1:N
    % PID Controller
    Error(N+1) = Velocity(t)-x_hat(2,t);
    
    Prop(N+1) = Error(N+1);
    Int(N+1)=(Error(N+1) + Error(N))*Ts/2;
    Integral(N+1)= sum(Int); 
    
    a(t) = Kp*Prop(N+1)+ Ki*Integral(N+1);

    % Plant Open Loop Simulation
    x(:,t+1) = F*x(:,t) + G*a(t);
    y(t) = H*x(:,t)+ noise(t); % Add noise to the measurement   
    
    % Estimate the Velocity using Kalman Filter
    [x_up, P_k{t+1}] = Kalman_Filter(x_hat(:,t), P_k{t}, y(t), F, H, Q, R);
    x_hat(:,t+1) = x_up;
end

%% Plot Data
maxfig
subplot(2,1,1)
plot(time,x(1,:),'-',time,x_hat(1,:),'--',time,[0 y],'--','Linewidth',2)
xlabel('Time (s)','fontweight','bold')
ylabel('Distance (m)','fontweight','bold')
grid on
legend('True Position','Estimated Position','Measurement','Location','northwest')
title('Estimated vs Real Position', 'FontSize', 14)
axes('position',[.68 .64 .2 .2])
box on
your_index = 58<time & time<65;
plot(time(your_index),x(1,your_index),time(your_index),x_hat(1,your_index),time(your_index),y(your_index),'Linewidth',2)
axis tight

subplot(2,1,2)
plot(time,x(1,:)-x_hat(1,:),time,x_hat(1,:)-[0 y],'Linewidth',1.5)
legend('True Position','Measurement','Location','northwest')
xlabel('Time (s)','fontweight','bold')
ylabel('Distance (m)','fontweight','bold')
grid on
title('Error', 'FontSize', 14)
saveas(gcf,'Position.png')

maxfig
subplot(2,1,1)
plot(time,[0 Velocity],time,x(2,:),time,x_hat(2,:),'Linewidth',2)
xlabel('Time (s)','fontweight','bold')
ylabel('Velocity (m/s)','fontweight','bold')
legend('Velocity Profile','True Velocity','Estimated Velocity','Location','northeast')
grid on
title('Estimated vs Real Velocity ', 'FontSize', 14)
subplot(2,1,2)
plot(time,[0 Velocity]-x(2,:),'Linewidth',1.5)
xlabel('Time (s)','fontweight','bold')
ylabel('Velocity (m/s)','fontweight','bold')
grid on
title('Error', 'FontSize', 14)
saveas(gcf,'Velocity.png')

maxfig
plot(time,a,'Linewidth',2)
xlabel('Time (s)','fontweight','bold')
ylabel('Acceleration (m/s^2)','fontweight','bold')
grid on
title('Acceleration - Controller Output', 'FontSize', 14)
saveas(gcf,'Acceleration.png')