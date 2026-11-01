%% Q1.6 - Filament Cooling Simulation
clear;
clc;
close all;

%% 1. Parameters
% Chosen filament parameters
rho = 1200;        % Density (kg/m^3)
cp = 1800;         % Specific heat capacity (J/kg/K)
k = 0.2;           % Thermal conductivity (W/m/K)
w = 0.0005;        % Filament width (m)

% Heat transfer coefficients
h_b = 80;          % Heat transfer with bed (W/m^2/K)
h_a = 10;          % Heat transfer with air (W/m^2/K)

% Temperatures
T_b = 80;          % Printing bed temperature (C)
T_e = 22;          % Environmental temperature (C)

% Ellipse parameters (1.4)
xc = 0.145;        % Centre x (m)
yc = 0.145;        % Centre y (m)
a = 0.065;         % Horizontal radius (m)
b = 0.035;         % Vertical radius (m)

% Chosen electrical parameters (1.5)
L_n = 0.002;       % Inductance of nozzle (H)
R_n = 14;          % Resistance of nozzle (Ohm)
C_n = 100e-6;      % Capacitance of nozzle (F)

% Chosen thermal parameters (1.5)
C_tn = 12;         % Thermal capacity of nozzle (J/K)
R_tn = 5;          % Thermal resistance of nozzle (K/W)

% Applied voltage
u_n = 24;          % Voltage (V)

% Simulation settings
N = 100;           % Number of filament segments
dt = 0.05;         % Time step (s)
T_print = 30;      % Printing time (s)
preheat = 300;     % Nozzle preheating time (s)
max_Cooling = 180;  % Maximum cooling time (s)
tolerance = 1;     % Temperature tolerance (C)


%% 2. Initial conditions

T_n0 = T_e;                     % Initial nozzle temperature
i_n0 = 0;                       % Initial electrical current
v_n0 = 0;                       % Initial capacitor voltage

z0 = [T_n0; i_n0; v_n0];        % Initial state vector

% Calculate the ellipse length
angle = linspace(0,2*pi,1001)';

x = xc + a*cos(angle);
y = yc + b*sin(angle);

distance = sqrt(diff(x).^2 + diff(y).^2);
L_total = sum(distance);

% Divide the filament into segments
ds = L_total/N;

% Position of each segment
s = ((1:N)' - 0.5)*ds;

% Simulation time
time = (0:dt:T_print+max_Cooling)';

% Initial filament temperatures
temp = NaN(N,1);            % NaN = segment not printed yet

% Store simulation results
T_history = NaN(N,length(time));
T_max = NaN(length(time),1);
T_min = NaN(length(time),1);

% Initially no filament has been printed
n_Printed = 0;
printed_Length = 0;


%% 3. Simulate the nozzle and filament

% Simulation time for the nozzle
time_nozzle = [0; preheat + (0:dt:T_print)'];

% Define the differential equations
model = @(t,z) nozzleODE(t, z, u_n, L_n, R_n, C_n, C_tn, R_tn, T_e);

% Solve the differential equations
options = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t_n,z_n] = ode15s(model,time_nozzle,z0,options);

% Nozzle temperature during printing
T_nozzle = z_n(2:end,1);

% Simulate printing and cooling
for j = 1:length(time)-1
    t = time(j);

    % 3.1. Calculate cooling of existing filament
    if n_Printed > 0
        % Current filament temperatures
        T = temp(1:n_Printed);
        % Temperatures of neighbouring segments
        if t < T_print
            % Open filament: insulated ends
            T_left = [T(1); T(1:end-1)];
            T_right = [T(2:end); T(end)];

        else
            % Closed filament: connected ends
            T_left = [T(end); T(1:end-1)];
            T_right = [T(2:end); T(1)];
        end

        conduction = (k/(rho*cp)) * (T_right-2*T+T_left)/ds^2;    % Heat conduction along the filament
        bed = (h_b/(rho*cp*w)) * (T_b-T);      % Heat exchange with the printing bed
        air = (3*h_a/(rho*cp*w)) * (T_e-T);    % Heat exchange with the surrounding air
        dT = conduction + bed + air;           % Temperature derivative

        temp(1:n_Printed) = T + dt*dT;          % Update temperatures (Euler method)
    end

    % 3.2. Add new filament during printing
    if t < T_print
        phi = 2*pi*(t+dt/2)/T_print;         % Current position along the ellipse
        speed = (2*pi/T_print) * sqrt((a*sin(phi))^2 + (b*cos(phi))^2);    % Calculate printing speed
        printed_Length = printed_Length + speed*dt;    % Calculate deposited length

        nNew = min(N,floor(printed_Length/ds));       % Calculate number of printed segments

        % Complete the ellipse at the end of printing
        if time(j+1) >= T_print
            nNew = N;
        end

        % Initialise newly printed segments
        if nNew > n_Printed
            temp(n_Printed+1:nNew) = T_nozzle(j+1);
        end

        n_Printed = nNew;    % Update number of printed segments
    end

    % 3.3. Save the temperatures
    T_history(:,j+1) = temp;
    if n_Printed > 0
        T_max(j+1) = max(temp(1:n_Printed));
        T_min(j+1) = min(temp(1:n_Printed));
    end

end


%% 4. Get the simulation results

% Calculate the time needed for uniform temperature
cooling_Time = NaN;

for j = 1:length(time)
    if time(j) >= T_print
        if T_max(j)-T_min(j) < tolerance
            cooling_Time = time(j)-T_print;
            break;
        end
    end
end

% Theoretical equilibrium temperature
T_eq = (h_b*T_b + 3*h_a*T_e)/(h_b + 3*h_a);


%% 5. Plot the results

figure;

% Plot 1: Filament temperature distribution
subplot(2,1,1)

h = imagesc(time,s,T_history);

% Hide segments that have not been printed
set(h,'AlphaData',~isnan(T_history));
set(gca,'YDir','normal');

colorbar;

xlabel('Time (s)');
ylabel('Position along filament (m)');
title('Filament Temperature Distribution');
xline(T_print,'k--');


% Plot 2: Maximum and minimum temperatures
subplot(2,1,2)

plot(time,T_max,'r','LineWidth',1.5);
hold on;
plot(time,T_min,'b','LineWidth',1.5);
yline(T_eq,'k--');

xlabel('Time (s)');
ylabel('Temperature (C)');
title('Filament Cooling');

legend('Maximum','Minimum','Equilibrium');
xline(T_print,'k--');
grid on;


%% 6. Cooling time

fprintf('Total filament length: %.3f m\n',L_total);
fprintf('Equilibrium temperature: %.2f C\n',T_eq);

if ~isnan(cooling_Time)
    fprintf('Cooling time after printing: %.2f s\n',cooling_Time);
else
    fprintf('Temperature not yet uniform.\n');
end


%% 7. ODE function - Nozzle dynamics (Q1.5)

function dz = nozzleODE(t,z,u_n,L_n,R_n,C_n,C_tn,R_tn,T_e)
    % 7.1. Current state variables
    T_n = z(1);
    i_n = z(2);
    v_n = z(3);

    % 7.2. Thermal dynamics
    P_heat = v_n^2/R_n;             % Heat generated by the resistor
    P_loss = (T_n-T_e)/R_tn;        % Heat lost to the environment
    dT_n = (P_heat-P_loss)/C_tn;    % Temperature derivative

    % 7.3. Electrical dynamics
    di_n = (u_n-v_n)/L_n;        % Current derivative (inductor)
    dv_n = (i_n-v_n/R_n)/C_n;    % Voltage derivative (capacitor)

    % 7.4. Return the three derivatives
    dz = [dT_n; di_n; dv_n];
end