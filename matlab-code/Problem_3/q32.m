%% Q3.2Drawing a Square
clear;
clc;
close all;


% motor parameters 
p.r_m = 0.009;     % Motor radius(m)
p.L_m = 0.006;       % Motor inductance(H)
p.R_m = 14;          % Motor resistance (Ohm)
p.J_m = 1.2e-5;     % Motor inertia (kg*m^2)
p.K_m = 0.055;       % Motor constant (Nm/A)
p.m = 0.090;         % Print head mass(kg)
p.mu_x = 0.007;      % Linear friction x (Ns/m)
p.mu_y = 0.005;      % Linear friction y (Ns/m)
p.mu_r = 0.006;      % Rolling friction (Ns/m)

% kinematicequations from Q1.2
p.J_eq = p.J_m + p.m*p.r_m^2/2;
p.d = p.r_m^2/4*(p.mu_x+p.mu_y) + p.mu_r*p.r_m^2;
p.c = p.r_m^2/4*(p.mu_x-p.mu_y);

% time constants after switching off v
p.tau_x = p.J_eq/(p.d + p.c + p.K_m^2/p.R_m);
p.tau_y = p.J_eq/(p.d - p.c + p.K_m^2/p.R_m);

% settings to make a square
p.U = 1;             % Applied voltage while driving (V)
a = 0.10;            % Side length (m)
p.eps = 0.01;        % Angular velocity tolerance (rad/s)

% Corners c_1 ... c5 starts at origin
p.corners = [0 a a 0 0;
             0 0 a a 0];

% Directions of the four sides: right, up, left, down
p.n = [1 0 -1  0;
       0 1  0 -1];


X = zeros(6,1);      % Head at rest in origin
q = 1;               % Discrete state: 1=D, 2=S, 3=E
k = 1;               % First side
t0 = 0;


%%  Simulation

t_all = [];  X_all = [];  stops = [];

while q ~= 3
    opts = odeset('RelTol',1e-8,'AbsTol',1e-10,'Events',@(t,X) guards(X,q,k,p));
    [t,Xs,te,Xe,~] = ode15s(@(t,X) squareSys(X,q,k,p),[t0 t0+10],X,opts);

    t_all = [t_all; t];
    X_all = [X_all; Xs];
    X = Xe(end,:)';
    t0 = te(end);

    % Discrete transition
    if q == 1                          
        q = 2;
    else
        stops = [stops, p.r_m/2*[X(1)+X(2); X(1)-X(2)]];
        if k < 4                       
            q = 1;
            k = k + 1;
        else                           
            q = 3;
        end
    end
end


%% results of simulation

% actual position of the print head 
x = p.r_m/2*(X_all(:,1)+X_all(:,2));
y = p.r_m/2*(X_all(:,1)-X_all(:,2));

% Distance between the stopping points and the corners
corner_err = sqrt(sum((stops - p.corners(:,2:5)).^2,1));


figure;

subplot(1,2,1)
plot(p.corners(1,:),p.corners(2,:),'k--','LineWidth',1.5);
hold on;
plot(x,y,'b','LineWidth',1.5);
legend('Desired','Simulated');
xlabel('x (m)');
ylabel('y (m)');
title('Square Trajectory');
axis equal;
xlim([-0.01 a+0.01]);
ylim([-0.01 a+0.01]);
grid on;

subplot(1,2,2)
plot(t_all,X_all(:,3),'r','LineWidth',1.5);
hold on;
plot(t_all,X_all(:,4),'b','LineWidth',1.5);
legend('\omega_l','\omega_r');
xlabel('Time (s)');
ylabel('Angular velocity (rad/s)');
title('Motor Angular Velocities');
grid on;


%% results

fprintf('Time constants: tau_x = %.4f s, tau_y = %.4f s\n',p.tau_x,p.tau_y);
fprintf('Square finished at t = %.2f s\n',t0);
fprintf('Corner errors: %.4f %.4f %.4f %.4f mm\n',1000*corner_err);


%% functions

% motor dynamics 
function dX = squareSys(X,q,k,p)
    


% decoupled inputs
    if q == 1
        u_xy = p.U*p.n(:,k);
    else
        u_xy = [0; 0];
    end
    u_l = u_xy(1) + u_xy(2);
    u_r = u_xy(1) - u_xy(2);
    w_l = X(3);  w_r = X(4); i_l = X(5);i_r = X(6);
    dw_l = (p.K_m*i_l - p.d*w_l - p.c*w_r)/p.J_eq;
    dw_r = (p.K_m*i_r - p.c*w_l - p.d*w_r)/p.J_eq;
    di_l = (u_l - p.R_m*i_l - p.K_m*w_l)/p.L_m;
    di_r = (u_r - p.R_m*i_r - p.K_m*w_r)/p.L_m;

    dX = [w_l; w_r; dw_l; dw_r; di_l; di_r];
end




% guards

function [value,isterminal,direction] = guards(X,q,k,p)
    
pos = p.r_m/2*[X(1)+X(2); X(1)-X(2)];
    vel = p.r_m/2*[X(3)+X(4); X(3)-X(4)];

    if q == 1
     
        d_k = p.n(:,k)'*(p.corners(:,k+1) - pos);
        l_k = p.n(1,k)*vel(1)*p.tau_x + p.n(2,k)*vel(2)*p.tau_y;
        value = d_k - l_k;
    else
        value = abs(X(3)) + abs(X(4)) - p.eps;
    end
    isterminal= 1;
    direction = -1;
end