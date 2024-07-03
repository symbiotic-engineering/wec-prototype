clear;clc;close all

lambda = 50; plot_hydro = false;
dof = 5; % RM5 or RM3
[w_scaled, mI_A_scaled, B_scaled, K_scaled, gamma_scaled] = coeffs(dof,lambda,plot_hydro);

if dof == 5
    K_scaled = 0.8580; % override since K_scaled is somehow negative
end
H = 0.01;

% parameters
p = struct('Kh',K_scaled, ...  % hydrodynamic stiffness
           'Ig',0.1,...     % moment of inertia of generator/powertrain
           'tau_max_Nm',4, 'motor_max_rpm',3000,... % motor max torque and speed
           'T_s',0.1,'T_d',0.1,'b',.1,...  % static and dynamic friction torques and viscous friction coefficient
           'dof',dof);

% set gear ratios and springs to sweep over
gear_ratio = [1 3 5]; % gear ratio range
if dof == 3
    spring_size = [0 1]; % graph won't work if spring size is scalar, so add 1 for now
elseif dof == 5
    spring_size = 1:1:10;  % constant force spring size [N-m]
end

% set frequencies to sweep over
g = 9.8;
depth = 1.36; % Oregon tank depth [m]
w_min_deepwater = sqrt(2*g./depth);
w_max = 10; % rad/s
omega_idx_min = find(w_scaled > w_min_deepwater,1,'first');
omega_idx_max = find(w_scaled < w_max,          1,'last');
omega_idxs = omega_idx_min : omega_idx_max;
omegas = w_scaled(omega_idxs);

is_acceptable_combo = zeros(length(gear_ratio),length(spring_size),length(omega_idxs));
max_motor_torque    = zeros(length(gear_ratio),length(spring_size),length(omega_idxs));
plot_timeseries = false;

[spring_mesh, omega_mesh] = meshgrid(spring_size,omegas);

for i = 1:length(gear_ratio)

    p.GR = gear_ratio(i); % gear ratio

    for j = 1:length(spring_size)

        p.T_spring = spring_size(j); % spring torque

        for k = 1:length(omega_idxs)
            omega_idx = omega_idxs(k);
            p.Bh = B_scaled(1,omega_idx); % hydrodynamic damping
            p.I = mI_A_scaled(1,omega_idx); % moment of inertia of flap
            p.w = w_scaled(1,omega_idx); % wave frequency and 
            p.Fh = H/2 * gamma_scaled(1,omega_idx); % exciting force amplitude

            % impedance matching
            p.Kp = p.I * p.w^2 - p.Kh;
            p.Bp = p.Bh;
            
            % ode inputs
            T = 2*pi/p.w;
            y0 = [0;0];
            yp0 = [0;0];
            tspan = [0 5*T];
            
            % ode solve
            options = odeset('MaxStep',T/10);
            odefun = @(t,y,yp)dynamics(t,y,yp,p);
            %[y0,yp0] = decic(odefun, 0, y0, [1,1], yp0, [0,0]);
            sol = ode15i(odefun,tspan,y0,yp0,options);
            t = linspace(tspan(1),tspan(end));
            try
                [y,yp] = deval(sol,t);
                [err,P,T_gen,T_fric,T_string] = dynamics(t,y,yp,p);
                
                if dof == 5 && any(T_string <= 0)
                    fprintf('UNACCEPTABLE GR %f and Spring Force %f and Frequency %f\n',gear_ratio(i),spring_size(j),w_scaled(1,omega_idx))
                else
                    is_acceptable_combo(i,j,k) = true;
                    max_motor_torque(i,j,k) = max(abs(T_gen));
                    fprintf('  ACCEPTABLE GR %f and Spring Force %f and Frequency %f\n',gear_ratio(i),spring_size(j),w_scaled(1,omega_idx))
    
                    if plot_timeseries
                        % plot solution
                        figure
                        plot(t,y,  t,T_gen, t,T_fric, t,T_string, t,P)
                        xlabel('Time (s)')
                        legend('$\theta$','$\dot{\theta}$','$T_{gen}$','$T_{fric}$','$T_{string}$', ...
                            'P','interpreter','latex','FontSize',14)
                        
                        figure
                        xlabel('Time (s)')
                        ylabel('Integration Error')
                        plot(t,err(1,:),t,err(2,:))
                        legend('err_T (Nm)','err_v (m/s)')
                    end
                end
            catch exception
                if strcmp(exception.identifier, 'MATLAB:deval:SolOutsideInterval')
                    fprintf('SIM FAILED   GR %f and Spring Force %f and Frequency %f\n',gear_ratio(i),spring_size(j),w_scaled(1,omega_idx))
                    is_acceptable_combo(i,j,k) = NaN;
                    max_motor_torque(i,j,k) = NaN;
                else
                    rethrow(exception);
                end
            end

        end
    end

    acceptable = squeeze(is_acceptable_combo(i,:,:))';
    figure
    pcolor(spring_mesh, omega_mesh, acceptable)
    colorbar
    xlabel('spring torque Nm')
    ylabel('frequency rad/s')
    title(['Acceptable combinations for preventing slackness for GR=' num2str(gear_ratio(i))])
    clim([0 1])

    torque = squeeze(max_motor_torque(i,:,:))';
    figure
    pcolor(spring_mesh, omega_mesh, torque)
    colorbar
    xlabel('spring torque Nm')
    ylabel('frequency rad/s')
    title(['Max Motor Torque for GR=' num2str(gear_ratio(i))])
    clim([0 p.tau_max_Nm])
end


% equation of motion
function [err,P,T_gen,T_fric,T_string] = dynamics(t,y,yp,p)

    % states
    th = y(1,:);
    th_dot = y(2,:);
    
    th_dot_next = yp(1,:);
    th_ddot = yp(2,:);
    
    % effective gear ratio
    alpha = 0; % fixme - should be a function of theta
    GR_eff = p.GR * cos(alpha);
    
    % hydro torques on flap
    T_exc = p.Fh * sin(p.w * t);
    T_hydro = p.Kh * th + p.Bh * th_dot;
    T_inertia = p.I * th_ddot;
    
    % string torque on drivetrain
    T_string = (T_inertia + T_hydro - T_exc) / GR_eff;
    if p.dof == 5
        T_string(T_string < 0) = 0;
    end
    
    % generator torque
    T_gen = -(p.Kp * th + p.Bp * th_dot) / GR_eff; % fixme
    T_gen = min(max(T_gen/p.tau_max_Nm, -1), 1) * p.tau_max_Nm;
    motor_rpm = GR_eff * th_dot * 60/(2*pi);
    T_gen(abs(motor_rpm) > p.motor_max_rpm) = 0;
    
    % friction
    zero_speed_idxs = ismembertol(th_dot,0);
    net_trq_without_fric = T_gen - T_string + p.T_spring;
    T_static = min(p.T_s,abs(net_trq_without_fric)) .* sign(net_trq_without_fric);
    T_dynamic = p.T_d * sign(th_dot) + p.b * GR_eff * th_dot;
    T_fric = zeros(size(T_gen));
    T_fric(zero_speed_idxs) = T_static(zero_speed_idxs);
    T_fric(~zero_speed_idxs) = T_dynamic(~zero_speed_idxs);
    
    % unbalanced torque on drivetrain
    T_unb = p.Ig * GR_eff * th_ddot - T_gen + T_fric + T_string - p.T_spring;
    
    err = [T_unb; th_dot - th_dot_next];
    
    % power
    P = T_gen .* th_dot * GR_eff;

end