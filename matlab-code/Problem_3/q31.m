%% Q3.1 Hybrid System
clear;
clc;
close all;


% motor parameters 
p.r_m = 0.009;       % Motor radius(m)
p.L_m = 0.006;       % Motor inductance (H)
p.R_m = 14;          % Motor resistance (Ohm)
p.J_m = 1.2e-5;      % Motor inertia (kg*m^2)
p.K_m = 0.055;       % Motor constant (Nm/A)
p.m = 0.090;         % Print head mass (kg)
p.mu_x = 0.007;      % Linear friction x (Ns/m)
p.mu_y = 0.005;      % Linear friction y (Ns/m)
p.mu_r = 0.006;      % Rolling friction (Ns/m)

% equations from Q1.2
p.J_eq = p.J_m + p.m*p.r_m^2/2;
p.d = p.r_m^2/4*(p.mu_x+p.mu_y) + p.mu_r*p.r_m^2;
p.c = p.r_m^2/4*(p.mu_x-p.mu_y);

% nozzle parameters 
p.L_n = 0.002;       % Inductance of nozzle (H)
p.R_n = 14;          % Resistance of nozzle (Ohm)
p.C_n = 100e-6;      % Capacitance of nozzle (F)
p.C_tn = 12;         % Thermal capacity of nozzle (J/K)
p.R_tn = 5;          % Thermal resistance of nozzle (K/W)
p.T_e = 22;          % Environmental temperature (C)
p.u_n = 24;          % Heater voltage (V)

% ellipse 
p.xc = 0.145;        % Centre x (m)
p.yc = 0.145;        % Centre y (m)
p.a = 0.065;         % Horizontal radius (m)
p.b = 0.035;         % Vertical radius (m)
p.T = 30;            % Time for one ellipse (s)

% hybrid parameters
p.s_sp = 600;        % Printing time of a new spool (s)
p.tau_w = 3;         % Wiping time (s)
p.tau_L = 5;         % Loading time (s)
p.T_l = 150;         % Low nozzle temperature (C)
p.T_h = 220;         % High nozzle temperature (C)
p.delta_p = 0.5e-3;  % Position tolerance (m)
p.delta_v = 2e-3;    % Velocity tolerance (m/s)
p.Kp = 20;           % Proportional gain (V/m)
p.p_w = [0; 0];      % Wipe tower position (m)

% memory variables ( changed by reset maps)
p.x_s = 0;           % Saved position x (m)
p.y_s = 0;           % Saved position y (m)
p.s_C = Inf;         % Colour change time (no colour change)
p.s_F = 7.5;         % Spool runs out after 7.5 s of printing

% the discrete states
names = {'q_P','q_M','q_W','q_O','q_L','q_H','q_R'};


%% 2.initial conditions f system

opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[~,z_n] = ode15s(@(t,z) nozzleODE(z,p.u_n,p),[0 300],[p.T_e; 0; 0],opts);

[Px,Py] = ellipseRef(0,p);
i_x = (p.J_eq*Px(3) + (p.d+p.c)*Px(2))/(p.r_m*p.K_m);
i_y = (p.J_eq*Py(3) + (p.d-p.c)*Py(2))/(p.r_m*p.K_m);

% State vector X 
X = [(Px(1)+Py(1))/p.r_m; (Px(1)-Py(1))/p.r_m;
     (Px(2)+Py(2))/p.r_m; (Px(2)-Py(2))/p.r_m;
     i_x+i_y; i_x-i_y; z_n(end,:)'; 0; 0];

q = 1;               % Start at printing state
t0 = 0;


%% 3. Simulation

t_all = [];  X_all = [];  q_all = [];  u_all = [];

while true
    
    opts = odeset(opts,'Events',@(t,X) guards(X,q,p));
    [t,Xs,te,Xe,ie] = ode15s(@(t,X) printerSys(X,inputs(X,q,p),q,p), ...
                             [t0 t0+600],X,opts);

  
    t_all = [t_all; t];
    X_all = [X_all; Xs];
    q_all = [q_all; q*ones(size(t))];
    for j = 1:length(t)
        u_all = [u_all; inputs(Xs(j,:)',q,p)'];
    end

    X = Xe(end,:)';
    t0 = te(end);

  
    if q == 1 && ie(end) == 2
        break;
    end


    fprintf('t = %6.2f s: %s -> ',t0,names{q});
    [q,X,p] = jump(q,X,p);
    fprintf('%s\n',names{q});
end

fprintf('Print finished at t = %.2f s\n',t0);


x = p.r_m/2*(X_all(:,1)+X_all(:,2));
y = p.r_m/2*(X_all(:,1)-X_all(:,2));

%  ellipse
s = linspace(0,p.T,500);
xd = p.xc + p.a*cos(2*pi*s/p.T);
yd = p.yc + p.b*sin(2*pi*s/p.T);


figure;
plot(xd,yd,'k--','LineWidth',1.5);
hold on;
plot(x,y,'b','LineWidth',1.2);
plot(p.p_w(1),p.p_w(2),'rs','MarkerSize',9,'LineWidth',1.5);
plot(p.x_s,p.y_s,'ro','MarkerSize',9,'LineWidth',1.5);
legend(' ellipse','Print head','Wipestation','Saved point','Location','northwest');
xlabel('x (m)');
ylabel('y (m)');
title('Print Head Trajectory');
axis equal;
xlim([-0.01 0.29]);
ylim([-0.01 0.29]);
grid on;

figure;

subplot(3,1,1)
stairs(t_all,q_all,'k','LineWidth',1.5);
yticks(1:7);
yticklabels(names);
xlabel('Time (s)');
ylabel('State');
title('Discrete State');
grid on;

subplot(3,1,2)
plot(t_all,X_all(:,7),'r','LineWidth',1.5);
hold on;
yline(p.T_l,'b--');
yline(p.T_h,'k--');
legend('T_n','T_l','T_h');
xlabel('Time (s)');
ylabel('Temperature (C)');
title('Nozzle Temp');
grid on;

subplot(3,1,3)
plot(t_all,u_all(:,1),'r','LineWidth',1.2);
hold on;
plot(t_all,u_all(:,2),'b','LineWidth',1.2);
plot(t_all,u_all(:,3),'k','LineWidth',1.2);
legend('u_l','u_r','u_n');
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Motor and Heater Voltages');
grid on;



function dX = printerSys(X,u,q,p)
    w_l = X(3);  w_r = X(4);  i_l = X(5);  i_r = X(6);

    dw_l = (p.K_m*i_l - p.d*w_l - p.c*w_r)/p.J_eq;
    dw_r = (p.K_m*i_r - p.c*w_l - p.d*w_r)/p.J_eq;
    di_l = (u(1) - p.R_m*i_l - p.K_m*w_l)/p.L_m;
    di_r = (u(2) - p.R_m*i_r - p.K_m*w_r)/p.L_m;

    dz_n = nozzleODE(X(7:9),u(3),p);

    if q == 1
        ds = 1;
    else
        ds = 0;
    end
    dtau = 1;

    dX = [w_l; w_r; dw_l; dw_r; di_l; di_r; dz_n; ds; dtau];
end

function u = inputs(X,q,p)
    pos = p.r_m/2*[X(1)+X(2); X(1)-X(2)];

    if q == 1

        [Px,Py] = ellipseRef(X(10),p);
        ref = [Px(1); Py(1)];
        u_ff = [feedforward(Px,p.d+p.c,p); feedforward(Py,p.d-p.c,p)];
    elseif q == 7
 
        ref = [p.x_s; p.y_s];  u_ff = [0; 0];
    else
    
        ref = p.p_w;  u_ff = [0; 0];
    end

    u_xy = u_ff + p.Kp*(ref-pos);
    u_l = u_xy(1) + u_xy(2);
    u_r = u_xy(1) - u_xy(2);

    % heater off while cooling andloading
    if q == 4 || q == 5
        u_n = 0;
    else
        u_n = p.u_n;
    end

    u = [u_l; u_r; u_n];
end

% guards 
function [value,isterminal,direction] = guards(X,q,p)
    pos = p.r_m/2*[X(1)+X(2);X(1)-X(2)];
    vel = p.r_m/2*[X(3)+X(4); X(3)-X(4)];
    T_n = X(7);  s = X(10); tau = X(11);

    switch q
        case 1  
            value = [min(p.s_C,p.s_F)-s; s-p.T];
            direction = [-1; 1];
        case 2  
            value = arrival(pos,vel,p.p_w,p);       direction = -1;
        case 3  
            value = tau - p.tau_w;                  direction = 1;
        case 4  
            value = T_n - p.T_l;                    direction = -1;
        case 5  
            value = tau - p.tau_L;                  direction = 1;
        case 6  
            value = T_n - p.T_h;                    direction = 1;
        case 7  
            value = arrival(pos,vel,[p.x_s; p.y_s],p);  direction = -1;
    end
    isterminal = ones(size(value));
end

% reset maps
function [q,X,p] = jump(q,X,p)
    switch q
        case 1  %save position, clear colour event
            p.x_s = p.r_m/2*(X(1)+X(2));
            p.y_s = p.r_m/2*(X(1)-X(2));
            if X(10) >= p.s_C - 1e-6
                p.s_C = Inf;
            end
            q = 2;
        case 2  
            X(11) = 0;
            q = 3;
        case 3  
            q = 4;
        case 4  
            X(11) = 0;
            q = 5;
        case 5  
            p.s_F = X(10) + p.s_sp;
            q = 6;
        case 6  
            q = 7;
        case 7 
            q = 1;
    end
end

function g = arrival(pos,vel,target,p)
    g = max(norm(pos-target)/p.delta_p, norm(vel)/p.delta_v) - 1;
end

% ellipse [position;velcity; acceleratio;jerk] 
function [Px,Py] = ellipseRef(s,p)
    w = 2*pi/p.T;
    Px = [p.xc + p.a*cos(w*s); -p.a*w*sin(w*s); -p.a*w^2*cos(w*s);  p.a*w^3*sin(w*s)];
    Py = [p.yc + p.b*sin(w*s);  p.b*w*cos(w*s); -p.b*w^2*sin(w*s); -p.b*w^3*cos(w*s)];
end

function u = feedforward(P,gam,p)
    u = p.L_m/(p.r_m*p.K_m)*(p.J_eq*P(4) + gam*P(3)) ...
      + p.R_m/(p.r_m*p.K_m)*(p.J_eq*P(3) + gam*P(2)) ...
      + p.K_m/p.r_m*P(2);
end

function dz = nozzleODE(z,u_n,p)
    T_n = z(1);  i_n = z(2);  v_n = z(3);
    dT_n = (v_n^2/p.R_n - (T_n-p.T_e)/p.R_tn)/p.C_tn;
    di_n = (u_n - v_n)/p.L_n;
    dv_n = (i_n - v_n/p.R_n)/p.C_n;
    dz = [dT_n; di_n; dv_n];
end