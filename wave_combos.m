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
Hs = linspace(0.01, .5, 100); % m, peak to peak
f = linspace(.42, 2, 100); % Hz

[Hs_mesh, f_mesh] = meshgrid(Hs,f);
A_mesh = Hs_mesh / 2;
w_mesh = 2*pi*f_mesh; % rad/s
T_mesh = 1./f_mesh; % s
k = w_mesh.^2 / g;
lambda = 2*pi./k;
v2_grav = g*lambda/(2*pi);
v2_surf = 2*pi*sigma./(lambda*rho);
v = sqrt(v2_grav + v2_surf); % phase velocity
Wb = rho * v.^2 * D / sigma;

% WECSim results RM3
X = Hs_mesh * .26 / 8;

P_wave = rho * g^2 * Hs_mesh.^2 .* T_mesh / (64*pi) * D;

much_less_than_factor = 10;

small_wave_amplitude_linear = A_mesh ./ lambda < 1 / much_less_than_factor;
small_body_amplitude_linear = X / D < 1 / much_less_than_factor;
breaking = Hs_mesh / h < 3/4;
deep_water = k * h > 2;
surface_tension_speed = v2_surf ./ v2_grav < 1/much_less_than_factor;
surface_tension_weber = Wb > much_less_than_factor;
friction = 0;

all_ok = small_wave_amplitude_linear & small_body_amplitude_linear ...
    & breaking & deep_water & surface_tension_speed & surface_tension_weber; % & friction

P_wave(~all_ok) = NaN;

[maxP,idx] = max(P_wave,[],'all');

figure
contourf(Hs_mesh, f_mesh, double(all_ok), [0 1])
hold on
plot(Hs_mesh(idx),f_mesh(idx),'ro','MarkerFaceColor','r')
xlabel('Hs (m)')
ylabel('f (Hz)')
title('Conditions Met')
colorbar

% deep water criteria solved for each tank depth
m_to_ft = 3.28;
h_schools = [1.5/m_to_ft .7 4/m_to_ft 8/m_to_ft];
schools = {'Cornell','UMaine','MIT','UNH'};
f_min = sqrt(2*g./h_schools)/(2*pi);

figure
contourf(Hs_mesh, f_mesh, P_wave)
hold on
for i=1:length(schools)
    plot([0 max(Hs)],[f_min(i) f_min(i)],'r--')
    text(max(Hs)/2, f_min(i)+.03,['\uparrow ', schools{i}],'Color','r')
end
xlabel('Hs (m)')
ylabel('f (Hz)')
title('Power (W)')
colorbar

