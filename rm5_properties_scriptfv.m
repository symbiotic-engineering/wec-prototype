clear all
%% DEFINING GEOMETRIC PARAMETERS AND CONSTANTS

% Constants
density_foam = 240;       % Density of material 1 (in kg/m^3)
density_sheet = 1180;     % Density of material 2 (in kg/m^3) polyprene sheet
gravity = 9.81;           % Acceleration due to gravity (in m/s^2)
density_water = 1000;     % kg/m^3

% Geometry and dimensions of the submerged part of flap
height = 0.17;                  % Width (in meters)
length = 0.36;                  % Length (in meters)
t_sheet = 0.00555625;           % Thickness of sheet (in meters)
t_foam = 0.0254;                % Thickness of foam (in meters)
foam_area = t_foam * length;    % Total foam area
sheet_area = t_sheet * length * 2;  % Total sheet area (includes both sides)
gap = 0.02;                     % Distance from shaft to start of flap

% Calculate the initial mass 
volume_foam = height * foam_area;
volume_sheet = height * sheet_area;
mass_foam = density_foam * volume_foam * 1.27; %%%% where is this coming from? (7/8/24)
mass_sheet = density_sheet * volume_sheet;
mass_total = mass_foam + mass_sheet;
disp(['mass is ' num2str(mass_total)])

%% FINDING THE CENTER OF BUOYANCY
centroid = height/2; % center of mass of submerged component

% Calculate the center of buoyancy as the weighted average of layer centroids
volume_total = volume_sheet + volume_foam;
disp(['volume is ' num2str(volume_total)])

% Sum the weighted centroids to find the center of buoyancy
center_of_buoyancy = centroid+gap;

% Display the center of buoyancy
disp(['Center of Buoyancy: ' num2str(center_of_buoyancy)]);

%NOTE THAT WE CARAE about Y
buoyancy_force = density_water * volume_total;

%% DEFINING CENTER OF MASS
from_submerged = 3.9/50;  % WEC-Sim scaled distance of COM from water line (meters)
center_of_mass = gap + height - from_submerged; % height above axis of rotation

% Display the center of mass
disp(['Center of Mass (x, y, z): ' num2str(center_of_mass)]);

%% K calc
desired_k = 0.653;
our_k = center_of_buoyancy * buoyancy_force - mass_total * gravity * center_of_mass;
disp(['Stiffness: ' num2str(our_k) ' ~ ' num2str(desired_k) '?'])

%% FINDING MOMENT OF INERTIA

% Assuming the object is rotated around the same axis without the gap
I_foam = density_foam*width*(t_sheet^3*height/12 + t_sheet*height^3/3);

a = t_foam/2;      % starting coordinate of foam in thickness direction
b = a + t_sheet;   % ending coordinate of foam in thickness direction
I_sheet = density_sheet*width*(b^3*height + b*height^3 - a^3*height - a*height^3);

% Adjust for the gap between flap and axis
I_foam_adjusted = I_foam + mass_foam*(gap)^2;
I_sheet_adjusted = I_sheet + mass_sheet*(gap)^2;

I_total = I_foam_adjusted + 2*I_sheet_adjusted;

% Display the total moment of inertia
disp(['Total Moment of Inertia ' num2str(I_total)])

% Call the replace_foam function
[rho_foam, rho_weight, size, r, t_foam, gravity] = deal(25, 0, 0.087, 0.09+gap, t_foam, gravity);
[dm, dI, dK] = replace_foam(rho_foam, rho_weight, size, r, t_foam, gravity);
new_K = our_k+dK;
I_strut = 3.22e-5;
new_I = I_total+dI+I_strut;
new_m = mass_total+dm;
% Display the changes in mass, moment of inertia, and stiffness
disp(['Change in Mass: ' num2str(dm)]);
disp(['Change in Moment of Inertia: ' num2str(dI)]);
disp(['Change in Stiffness: ' num2str(dK)]);
disp(['New Stiffness:' num2str(new_K)]);
disp(['New Inertia: ' num2str(new_I)]);
disp(['New Mass: ' num2str(new_m)]);

function [dm, dI, dK] = replace_foam(rho_foam, rho_weight, size, r, t_foam, g)
    % Function definition for replace_foam
    I_new = add_weight(rho_weight, size, r, t_foam);
    I_old = add_weight(rho_foam, size, r, t_foam);
    dm = I_new - I_old;
    dI = I_new - I_old;
    dK = -dm * g;
end

function [I, m] = add_weight(rho, size, r, t_foam)
    % Function definition for add_weight
    m = rho * size^2 * t_foam;
    I = (1/12) * m * size^2;
    I = I + m * r^2;
end
