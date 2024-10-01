
function [w_scaled, A_scaled, B_scaled, K_scaled, gamma_scaled] = coeffs(dof,lambda,plot_on)
% finding hydro coefficients after Froude scaling

if nargin == 0
    dof = 5;
    plot_on = true;
    lambda = 50; % Froude scaling factor
    close all
end

[A,B,K,gamma,~,g,w] = extractData(dof);

% full scale parameters
if dof == 5
    m_I = 1850000;                  % moment of inertia [kg-m^2]
elseif dof == 3
    m_I = 725833;                   % RM3 float mass [kg]
end

% full scale reactive control
B_pto = B;
K_pto = (m_I + A).*w.^2 - K;

% Froude scaling
if dof == 5
    expo_B = 4.5;
    expo_K = 4;
    expo_A = 5;
    expo_G = 3;
elseif dof == 3
    expo_B = 2.5;
    expo_K = 2;
    expo_A = 3;
    expo_G = 2;
end
expo_w = -1/2;

B_scaled = B./(lambda^expo_B);
K_scaled = K./(lambda^expo_K);
B_pto_scaled = B_pto./(lambda^expo_B);
K_pto_scaled = K_pto./(lambda^expo_K);
mI_A_scaled = (m_I + A)./(lambda^expo_A);
A_scaled = A./(lambda^expo_A);
w_scaled = w./(lambda^expo_w);
gamma_scaled = gamma./(lambda^expo_G);

if plot_on
    % full scale transfer function
    numerator = gamma;
    denominator = (-1*w.^2 .*(m_I + A)) + (w.*(B+B_pto)) + (K + K_pto);
    %denominator = (-1.*(m_I + A).*w.^2 + K + K_pto).^2 + ((B + B_pto).*w).^2;
    RAO = numerator./(denominator); % WEC amplitude per unit wave amplitude full scale
    WEC_amp_mag_over_H = RAO * 1/2; % WEC amplitude per unit wave height full scale
    WEC_amp_mag_over_H_scaled = WEC_amp_mag_over_H; % stays constant over Froude scaling

    % scaled powertrain torque
    H_scaled = 0.01;            % wave height [m] used in wave tank
    T_powertrain = H_scaled * WEC_amp_mag_over_H_scaled .* sqrt(K_pto_scaled.^2 + (B_pto_scaled.*w_scaled).^2);%./gear_ratios';
    
    % deep water condition
    depth = 1.36; % Oregon tank depth [m]
    w_min_deepwater = sqrt(2*g./depth); % w^2/g * depth > 2

    % plot RAO
    figure
    plot(w,RAO)
    hold on
    plot([w_min_deepwater,w_min_deepwater],[0 60],'--')
    xlim([0 10])
    ylim([-1 1])
    xlabel('scaled omega')
    ylabel('RAO = WEC amplitude per unit wave amplitude')
    legend('RAO','Min Frequency for Deep Water')

    % plot scaled hydro coeffs
    figure
    plot(w_scaled,A_scaled)
    xlabel('scaled omega')
    hold on
    plot(w_scaled,B_scaled)
    xlabel('scaled omega')
    legend('scaled added mass','scaled damping')

    % plot powertrain torque at WEC
    figure
    plot(w_scaled,T_powertrain)
    hold on
    plot([w_min_deepwater,w_min_deepwater],[0 30],'--')
    xlim([0 10])
    ylim([0 30])
    xlabel('Scaled \omega [rad/s]')
    ylabel('Scaled Powertrain Torque on Flap [N-m]')
    legend(['Torque for H=',num2str(H_scaled)],'Min Frequency for Deep Water')
end

end
