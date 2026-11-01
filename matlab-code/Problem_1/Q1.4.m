%% Q1.4 - Drawing Shapes
clear;
clc;
close all;

%% 1. Parameters
% Chosen printer parameters
r_m = 0.009;       % Motor radius (m)
L_m = 0.006;       % Motor inductance (H)
R_m = 14;          % Motor resistance (Ohm)
J_m = 1.2e-5;      % Motor inertia (kg*m^2)
K_m = 0.055;       % Motor constant (Nm/A)
m = 0.090;         % Print head mass (kg)

% Bearing friction coefficients
mu_x = 0.007;      % Linear friction x (Ns/m)
mu_y = 0.005;      % Linear friction y (Ns/m)
mu_r = 0.006;      % Rolling friction (Ns/m)

% Printing area
dx = 0.29;         % Width (m)
dy = 0.29;         % Height (m)

% Simulation settings
shape = 1;         % 1 = Ellipse, 2 = Heart
T = 30;            % Printing time (s)

% Equations from Q1.2
J_eq = J_m + m*r_m^2/2;
d = r_m^2/4*(mu_x+mu_y) + mu_r*r_m^2;
c = r_m^2/4*(mu_x-mu_y);

%% 2. Desired trajectory and initial conditions

% Simulation time
time = linspace(0,T,1001)';

% Angle along the shape
angle = 2*pi*time/T;

% Centre of the printing bed
xc = dx/2;
yc = dy/2;

% Desired trajectory
if shape == 1

    % Ellipse
    xd = xc + 0.065*cos(angle);
    yd = yc + 0.035*sin(angle);

else

    % Heart
    xd = xc + 0.060*sin(angle).^3;
    yd = yc + 0.045*(cos(angle)-0.5*cos(2*angle));

end

% Desired motor angles (Q1.1)
theta_L = (xd+yd)/r_m;
theta_R = (xd-yd)/r_m;

% Desired motor angular velocities
omega_L = gradient(theta_L,time);
omega_R = gradient(theta_R,time);

% Desired motor angular accelerations
acc_L = gradient(omega_L,time);
acc_R = gradient(omega_R,time);

% Required motor currents (Q1.2)
i_L = (J_eq*acc_L + d*omega_L + c*omega_R)/K_m;
i_R = (J_eq*acc_R + c*omega_L + d*omega_R)/K_m;

% Required motor voltages (Q1.2)
u_L = L_m*gradient(i_L,time) + R_m*i_L + K_m*omega_L;
u_R = L_m*gradient(i_R,time) + R_m*i_R + K_m*omega_R;

% Initial motor angles
theta_L0 = theta_L(1);
theta_R0 = theta_R(1);

% Initial motor angular velocities
omega_L0 = omega_L(1);
omega_R0 = omega_R(1);

% Initial electrical currents
i_L0 = i_L(1);
i_R0 = i_R(1);

% Initial state vector
z0 = [theta_L0; theta_R0; omega_L0; omega_R0; i_L0; i_R0];

%% 3. Simulate the printer

% Define the differential equations
model = @(t,z) printerODE(t, z, time, u_L, u_R, L_m, R_m, J_eq, K_m, d, c);

% Solve the differential equations
[t,z] = ode15s(model,time,z0);

%% 4. Get the simulation results

% Motor angles
theta_L = z(:,1);
theta_R = z(:,2);

% Motor angular velocities
omega_L = z(:,3);
omega_R = z(:,4);

% Motor currents
i_L = z(:,5);
i_R = z(:,6);

% Actual position of the print head
x = r_m/2*(theta_L+theta_R);
y = r_m/2*(theta_L-theta_R);

%% 5. Plot the results
figure;

% Plot 1: Desired and simulated trajectory
subplot(1,2,1)

plot(xd,yd,'k--','LineWidth',2);
hold on;
plot(x,y,'b','LineWidth',1.5);

legend('Desired','Simulated');
xlabel('x (m)');
ylabel('y (m)');
title('Printer Trajectory');
axis equal;
xlim([0 dx]);
ylim([0 dy]);
grid on;

% Plot 2: Motor voltages
subplot(1,2,2)

plot(time,u_L,'r','LineWidth',1.5);
hold on;
plot(time,u_R,'b','LineWidth',1.5);

legend('Left motor','Right motor');
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Motor Voltages');
grid on;

%% 6. Maximum tracking error

error = sqrt((xd-x).^2+(yd-y).^2);      % Difference between desired and actual position
fprintf('Maximum tracking error: %.3f mm\n',max(error)*1000);

%% 7. ODE function - Printer dynamics (Q1.2)

function dz = printerODE(t,z,time,u_L,u_R,L_m,R_m,J_eq,K_m,d,c)
% 7.1. Motor voltages at the current time
u_l = interp1(time,u_L,t);
u_r = interp1(time,u_R,t);

% 7.2. Current state variables
theta_l = z(1);
theta_r = z(2);

omega_l = z(3);
omega_r = z(4);

i_l = z(5);
i_r = z(6);

% 7.3. Mechanical dynamics
% Angular velocity derivatives (motor angles)
dtheta_l = omega_l;
dtheta_r = omega_r;

% Angular acceleration derivatives
domega_l = (K_m*i_l-d*omega_l-c*omega_r)/J_eq;
domega_r = (K_m*i_r-c*omega_l-d*omega_r)/J_eq;

% 7.4. Electrical dynamics
di_l = (u_l-R_m*i_l-K_m*omega_l)/L_m;       % Current derivative (left motor)
di_r = (u_r-R_m*i_r-K_m*omega_r)/L_m;       % Current derivative (right motor)

% 7.5. Return the six derivatives
dz = [dtheta_l; dtheta_r; domega_l; domega_r; di_l; di_r];
end
