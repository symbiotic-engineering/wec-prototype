clear;clc;close all

% note: these dynamics are valid only for regular waves, since I haven't
% implemented the convolution integral (memory effect).

lambda = 50; plot_hydro = false;
dof = 3;%5; % RM5 or RM3

RM5_spring_string = false; % whether RM5 uses a string-spring PTO

H = [0.02,0.06,0.10,0.136]; % wave heights - from Olivia slack message 8/1/24
T_amp = 1:4; % torque amplitude for radiation tests

m_rotor = .354;
r_rotor = .087/2;
I_g = 1/2 * m_rotor * r_rotor^2; % moment of inertia of generator/powertrain
% parameters
p = struct('Ig',I_g,...
           'H',H,...
           'tau_max_Nm',4, 'motor_max_rpm',3000,... % motor max torque and speed
           'T_s',0.1,'T_d',0.1,'b',.1,...  % static and dynamic friction torques and viscous friction coefficient
           'dof',dof,'string_spring',RM5_spring_string);

% set gear ratios and springs to sweep over
if dof==5
    gear_ratio = [17 18]; % gear ratio range
elseif dof==3
    pinion_radius = [0.01 0.02 0.08 0.09]; % m
    gear_ratio = 1./pinion_radius; % 1/m
end
if dof == 3 || ~RM5_spring_string
    spring_size = [0 1]; % graph won't work if spring size is scalar, so add 1 for now
elseif dof == 5 && RM5_spring_string
    spring_size = 1:1:10;  % constant force spring size [N-m]
end

% set frequencies to sweep over
% g = 9.8;
% depth = 1.36; % Oregon tank depth [m]
% w_min_deepwater = sqrt(2*g./depth);
% w_max = 10; % rad/s
% omega_idx_min = find(w_scaled > w_min_deepwater,1,'first');
% omega_idx_max = find(w_scaled < w_max,          1,'last');
% omega_idxs = omega_idx_min : omega_idx_max;
omega_test = [7.07, 8.49, 9.83]; % override with actual test values

powered = true;
inner_loop_qty = H;
run_sweep(powered,gear_ratio,spring_size,omega_test,inner_loop_qty,lambda,plot_hydro,p);

powered = false;
inner_loop_qty = T_amp; 
run_sweep(powered,gear_ratio,spring_size,omega_test,inner_loop_qty,lambda,plot_hydro,p);

function [] = run_sweep(powered,gear_ratio,spring_size,omega_test,inner_loop_qty,lambda,plot_hydro,p)
    [w_scaled, A_scaled, B_scaled, K_scaled, gamma_scaled] = coeffs(p.dof,lambda,plot_hydro);
    
    if p.dof == 5
        K_scaled = 0.8580; % override since K_scaled is somehow negative
        I_scaled = 0.0372; % override to match actual ptype moment of inertia
        mI_A_scaled = A_scaled + I_scaled;
    elseif p.dof == 3
        m_scaled = 1.208;
        mI_A_scaled = A_scaled + m_scaled;
    end
    w_diff = abs(w_scaled - omega_test');
    [~,omega_idxs] = find(w_diff == min(w_diff,[],2));
    omegas = w_scaled(omega_idxs);

    if powered
        powered_title = 'powered';
        inner_loop_name = 'H - regular wave height (m)';
    else
        powered_title = 'forced oscillation';
        inner_loop_name = 'Generator torque amplitude (Nm)';
    end

    is_acceptable_combo = zeros(length(gear_ratio),length(spring_size),length(omega_idxs),length(inner_loop_qty));
    max_motor_torque    = zeros(length(gear_ratio),length(spring_size),length(omega_idxs),length(inner_loop_qty));
    plot_timeseries = false;
    
    [spring_mesh, omega_mesh] = meshgrid(spring_size,omegas);
    
    p.Kh = K_scaled; % hydrodynamic stiffness
    for i = 1:length(gear_ratio)
    
        p.GR = gear_ratio(i); % gear ratio
    
        for j = 1:length(spring_size)
    
            p.T_spring = spring_size(j); % spring torque
    
            for k = 1:length(omega_idxs)
                omega_idx = omega_idxs(k);
                p.Bh = B_scaled(1,omega_idx); % hydrodynamic damping
                p.I = mI_A_scaled(1,omega_idx); % moment of inertia of flap
                p.w = w_scaled(1,omega_idx); % wave frequency
                
                for inner_idx = 1:length(inner_loop_qty)
                    if powered
                        % impedance matching
                        p.Kp = p.I * p.w^2 - p.Kh;
                        p.Bp = p.Bh;
                        H = inner_loop_qty(inner_idx);
                        p.Fh = H/2 * gamma_scaled(1,omega_idx); % exciting force amplitude
                        odefun = @(t,y,yp)dynamics_power(t,y,yp,p);
                    else
                        p.T_gen_amplitude = inner_loop_qty(inner_idx);
                        odefun = @(t,y,yp)dynamics_radiation(t,y,yp,p);
                    end
                
                    [is_acceptable_combo(i,j,k,inner_idx), ...
                        max_motor_torque(i,j,k,inner_idx)] = run_sim(odefun,p,plot_timeseries);
                end
            end
        end
    
        acceptable = squeeze(is_acceptable_combo(i,:,:))';

        acceptable_title = ['Acceptable combinations for preventing slackness for GR=' num2str(gear_ratio(i)),', ' powered_title];
        plot_results(spring_size,omegas,acceptable,acceptable_title,[0 1])
    
        torque = squeeze(max_motor_torque(i,:,:))';
        torque_title = ['Max Motor Torque for GR=' num2str(gear_ratio(i)),', ' powered_title];
        plot_results(spring_size,omegas,torque,torque_title,[0 p.tau_max_Nm])
        
    end
end

function plot_results(x,y,z,my_title,z_lim)
    figure
    imagesc(x, y, z)
    colorbar
    xlabel('spring torque Nm')
    ylabel('frequency rad/s')
    title(my_title)
    clim(z_lim)
    draw_lines(x,y)
end

function [is_acceptable, max_motor_torque] = run_sim(odefun,p,plot_timeseries)
    % ode inputs
    T = 2*pi/p.w;
    y0 = [0;0];
    yp0 = [0;0];
    tspan = [0 5*T];
    
    % ode solve
    options = odeset('MaxStep',T/10);
    
    %[y0,yp0] = decic(odefun, 0, y0, [1,1], yp0, [0,0]);
    sol = ode15i(odefun,tspan,y0,yp0,options);
    t = linspace(tspan(1),tspan(end));
    try
        [y,yp] = deval(sol,t);
        [err,P,T_gen,T_fric,T_string] = odefun(t,y,yp);
        
        if p.dof == 5 && p.string_spring && any(T_string <= 0)
            is_acceptable = false;
            max_motor_torque = 0;
            %fprintf('UNACCEPTABLE GR %f and Spring Force %f and Frequency %f\n',gear_ratio(i),spring_size(j),w_scaled(1,omega_idx))
        else
            is_acceptable = true;
            max_motor_torque = max(abs(T_gen));
            %fprintf('  ACCEPTABLE GR %f and Spring Force %f and Frequency %f\n',gear_ratio(i),spring_size(j),w_scaled(1,omega_idx))

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
            %fprintf('SIM FAILED   GR %f and Spring Force %f and Frequency %f\n',gear_ratio(i),spring_size(j),w_scaled(1,omega_idx))
            is_acceptable = NaN;
            max_motor_torque = NaN;
        else
            rethrow(exception);
        end
    end
end

function draw_lines(spring_size,omegas)
    hold on
    d_sp = diff(spring_size(1:2))/2;
    d_w = diff(omegas(1:2))/2;
    for sp = spring_size
        plot(d_sp+[sp sp],[min(omegas)-d_w max(omegas)+d_w],'w','LineWidth',2)
    end
    for w = omegas
        plot([min(spring_size)-d_sp max(spring_size)+d_sp],d_w+[w w],'w','LineWidth',2)
    end
end

% dynamics for forced oscillation radiation test
function [err,P,T_gen,T_fric,T_string] = dynamics_radiation(t,y,yp,p)
    
    T_gen_cmd = p.T_gen_amplitude * sin(p.w * t);
    T_exc = 0;

    [err,P,T_gen,T_fric,T_string] = dynamics(T_exc,T_gen_cmd,t,y,yp,p);

end

% dynamics for impedance controlled power generation
function [err,P,T_gen,T_fric,T_string] = dynamics_power(t,y,yp,p)

    % states
    th = y(1,:);
    th_dot = y(2,:);
    
    % effective gear ratio
    alpha = 0; % fixme - should be a function of theta
    GR_eff = p.GR * cos(alpha);
    
    T_gen_cmd = -(p.Kp * th + p.Bp * th_dot) / GR_eff; % fixme

    % hydro excitaton torques on flap
    T_exc = p.Fh * sin(p.w * t);

    [err,P,T_gen,T_fric,T_string] = dynamics(T_exc,T_gen_cmd,t,y,yp,p);

end

% shared dynamics function - equation of motion
function [err,P,T_gen,T_fric,T_string] = dynamics(T_exc,T_gen_cmd,t,y,yp,p)
    % states
    th = y(1,:);
    th_dot = y(2,:);

    th_dot_next = yp(1,:);
    th_ddot = yp(2,:);

    % effective gear ratio
    alpha = 0; % fixme - should be a function of theta
    GR_eff = p.GR * cos(alpha);

    % hydro and interial torques
    T_hydro = p.Kh * th + p.Bh * th_dot;
    T_inertia = p.I * th_ddot;
    
    % string torque on drivetrain
    T_string = (T_inertia + T_hydro - T_exc) / GR_eff;
    if p.dof == 5 && p.string_spring
        T_string(T_string < 0) = 0;
    end
    
    % generator torque limiting
    T_gen = min(max(T_gen_cmd/p.tau_max_Nm, -1), 1) * p.tau_max_Nm;
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
