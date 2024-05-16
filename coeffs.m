% finding hydro coefficients for OSWEC to be used for scaling
addpath(genpath(pwd));
hydro = struct();
hydro = readWAMIT(hydro,'oswec.out',[]); % function from WECSim

[A,B,K,gamma,rho,g,w] = extractData(hydro);

I = 1850000;                % moment of inertia [kg-m^2]
H_s = 1;
lambda = 36;                % Froude scaling factor
beta = H_s/0.029;           % wave height ratio

B_pto = B;
K_pto = (I + A).*w.^2 - K;

numerator = gamma.*(H_s/2);
denominator = (-1.*(I + A).*w.^2 + K + K_pto).^2 + ((B + B_pto).*w).^2;

theta_mag = numerator./sqrt(denominator);
mterm = (I + A).*(w.^2).*theta_mag;
bterm = (B + B_pto).*w.*theta_mag;
kterm = (K + K_pto).*theta_mag;
T_body = mterm + bterm + kterm;

% gear_ratios = 1:1:8;
% T_motor = T_body./gear_ratios';     % excitation
% [X,Y] = meshgrid(w,gear_ratios);
% figure(10)
% plot(w,T_motor)

figure(5)
plot(w,T_body);
xlabel('\omega [rad/s]')
ylabel('Body Torque [N-m]')

B_scaled = B./(lambda^(4.5));
new_K = 0.8580;

K_scaled = new_K;%K./(lambda^4);
B_pto_scaled = B_pto./(lambda^(4.5));
K_pto_scaled = K_pto./(lambda^4);
I_A_scaled = (I + A)./(lambda^5);
A_scaled = A./(lambda^5);
w_scaled = w.*sqrt(lambda);
theta_mag_scaled = theta_mag.*(lambda/beta);

plot(w_scaled,A_scaled)
xlabel('scaled omega')
ylabel('scaled added mass')
plot(w_scaled,B_scaled)
xlabel('scaled omega')
ylabel('scaled damping')

F_powertrain = sqrt((K_pto_scaled.*theta_mag_scaled).^2 + (B_pto_scaled.*w_scaled.*theta_mag_scaled).^2);%./gear_ratios';
%F_powertrain = sqrt((K_pto_scaled.*theta_mag_scaled).^2 + (B_pto_scaled.*w_scaled).^2);%./gear_ratios';
figure(30)
plot(w_scaled,F_powertrain)
xlabel('Scaled \omega [rad/s]')
ylabel('Scaled Powertrain Torque [N-m]')
legend

% proto_motor_max = 4;        % [N-m]
% excess_torque = proto_motor_max - F_powertrain;
% figure(40)
% plot(w_scaled,excess_torque)

mterm_scaled = I_A_scaled.*(w_scaled.^2).*theta_mag_scaled;
bterm_scaled = (B_scaled).*w_scaled.*theta_mag_scaled;%(B_scaled + B_pto_scaled).*w_scaled.*theta_mag_scaled;
kterm_scaled = (K_scaled).*theta_mag_scaled;%(K_scaled + K_pto_scaled).*theta_mag_scaled;

figure(57)
hold on
plot(w_scaled,mterm_scaled)
plot(w_scaled,bterm_scaled)
plot(w_scaled,kterm_scaled)
xlabel('scaled omega')
ylabel('force component magnitude')
legend('mass term','damping term','stiffness term')

T_body_scaled = mterm_scaled + bterm_scaled + kterm_scaled; %%% used as input 

length = 0.38;        % Length (in meters)
x_motion = length.*sin(theta_mag_scaled);
F_hydro = T_body_scaled./x_motion;
plot(x_motion)

T_motor_scaled = T_body_scaled;%./gear_ratios';

% to find actual motor torque it's sum of squares square rooted
% MDOCEAN DOES THIS!!!! HURRAH
figure(15)
plot(w_scaled,T_motor_scaled)
