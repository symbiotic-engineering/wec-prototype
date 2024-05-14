clear
close all
load('hydro_data.mat');

% parameters
p = struct('Kh',new_K, ...  % hydrodynamic stiffness
           'Ig',0.1,...     % moment of inertia of generator/powertrain
           'tau_max_Nm',4, 'motor_max_rpm',3000,... % motor max torque and speed
           'T_s',0.1,'T_d',0.1,'b',.1);  % static and dynamic friction torques and viscous friction coefficient

%  p = struct('Kh',-1, 'Bh',1, ...  % hydrodynamic stiffness and damping
% 'I',0.14,'Ig',0.1,...     % moment of inertia of flap and generator/powertrain
% 'w',5, 'Fh',3, ...   % wave frequency and exciting force amplitude
% 'GR',2,'T_spring',10, ...      % gear ratio, spring torque
% 'tau_max_Nm',4, 'motor_max_rpm',3000,... % motor max torque and speed
% 'T_s',0.1,'T_d',0.1,'b',.1);  % static and dynamic friction torques and viscous friction coefficient

gear_ratio = [5 10]; % gear ratio range
spring_size = 8:1:20;  % constant force spring size [N-m]
omega_idxs = 40:1:50;    % location in w array of freqs 10-11.5 rad/s

is_acceptable_combo = false(length(gear_ratio),length(spring_size),length(omega_idxs));
plot_timeseries = false;

omegas = w_scaled(omega_idxs);
[spring_mesh, omega_mesh] = meshgrid(spring_size,omegas);

for i = 1:length(gear_ratio)

    p.GR = gear_ratio(i); % gear ratio

    for j = 1:length(spring_size)

        p.T_spring = spring_size(j); % spring torque

        for k = 1:length(omega_idxs)
            omega_idx = omega_idxs(k);
            p.Bh = B_scaled(1,omega_idx); % hydrodynamic damping
            p.I = I_A_scaled(1,omega_idx); % moment of inertia of flap
            p.w = w_scaled(1,omega_idx); % wave frequency and 
            p.Fh = T_body_scaled(1,omega_idx); % exciting force amplitude

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
            [y,yp] = deval(sol,t);
            [err,P,T_gen,T_fric,T_string] = dynamics(t,y,yp,p);
            
            if any(T_string <= 0)
                warning('string has gone slack')
            else
                is_acceptable_combo(i,j,k) = true;
                fprintf('ACCEPTABLE GR %f and Spring Force %f and Frequency %f\n',gear_ratio(i),spring_size(j),w_scaled(1,omega_idx))

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
        end
    end

    acceptable = is_acceptable_combo(i,:,:);
    figure
    pcolor(spring_mesh, omega_mesh, double(squeeze(acceptable))')
    colorbar
    xlabel('spring torque Nm')
    ylabel('frequency rad/s')
    title(['Acceptable combinations for preventing slackness for GR=' num2str(gear_ratio(i))])
    clim([0 1])
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
    T_string(T_string <= 0) = 0;
    
    % generator torque
    T_gen = -(p.Kp * th + p.Bp * th_dot) / GR_eff; % fixme
    %T_gen = sqrt((p.Kp * th).^2 + (p.Bp * th_dot).^2) / GR_eff;
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