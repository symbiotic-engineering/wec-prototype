% This script shows what combinations of wave height and period can meet 
% the assumptions of linear wave theory in a wave tank

clear
close all

% physical parameters
g = 9.8; % gravity
rho = 1000; % density water
sigma = 0.0728; % surface tension water

% wave tank parameters
D = .3; % diameter of WEC, based on what fits in width of wave tank
h = 3; % water depth (here made artificially high so that whole plot shows)

% all numbers represent tank scale not full scale
H = linspace(0.01, .2, 100); % m, peak to peak
f = linspace(.42, 2, 100); % Hz

[H_mesh, f_mesh] = meshgrid(H,f); % this assumes regular wave, not irregular (Hs =/= H)
A_mesh = H_mesh / 2; % amplitude [m]
w_mesh = 2*pi*f_mesh; % frequency [rad/s]
T_mesh = 1./f_mesh; % period [s]
k = w_mesh.^2 / g; % wavenumber in deep water [1/m]
lambda = 2*pi./k; % wavelength [m]
v2_grav = g*lambda/(2*pi); % v^2 due to gravity [m^2/s^2]
v2_surf = 2*pi*sigma./(lambda*rho); % v^2 due to surface tension [m^2/s^2]
v = sqrt(v2_grav + v2_surf); % phase velocity [m/s]
Wb = rho * v.^2 * D / sigma; % Weber number [-]

% WECSim results RM3
X = H_mesh * .26 / 8; % wec amplitude [m]

P_wave = rho * g^2 * H_mesh.^2 .* T_mesh / (32*pi) * D; % power in regular wave

% see which waves satisfy assumptions
much_less_than_factor = 10;

small_wave_amplitude_linear = A_mesh ./ lambda < 1 / much_less_than_factor;
small_body_amplitude_linear = X / D < 1 / much_less_than_factor;
breaking = H_mesh / h < 3/4;
deep_water = k * h > 2;
surface_tension_speed = v2_surf ./ v2_grav < 1/much_less_than_factor;
surface_tension_weber = Wb > much_less_than_factor;
friction = 0; % to be implemented

all_ok = small_wave_amplitude_linear & small_body_amplitude_linear ...
    & breaking & deep_water & surface_tension_speed & surface_tension_weber; % & friction

P_wave(~all_ok) = NaN;

[maxP,idx] = max(P_wave,[],'all');

figure
contourf(H_mesh, f_mesh, double(all_ok), [0 1])
hold on
plot(H_mesh(idx),f_mesh(idx),'ro','MarkerFaceColor','r')
xlabel('Hs (m)')
ylabel('f (Hz)')
title('Conditions Met')
colorbar

% deep water criteria solved for each tank depth
ft_to_m = 1/3.28;
h_schools = [1.5*ft_to_m, 0.7,     4*ft_to_m, 8*ft_to_m, 1.36];
schools =  {'Cornell',   'UMaine', 'MIT',     'UNH',     'Oregon'};
f_min = sqrt(2*g./h_schools)/(2*pi);

figure
contourf(H_mesh, f_mesh, P_wave)
hold on
for i=1:length(schools)
    plot([0 max(H)],[f_min(i) f_min(i)],'r--')
    text(max(H)/2, f_min(i)+.03,['\uparrow ', schools{i}],'Color','r')
end
xlabel('H (m)')
ylabel('f (Hz)')
title('Power (W)')
colorbar

