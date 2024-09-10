clear
close all

clear; clc;
% parameters
p = struct('Kh',1, 'Bh',1, ...  % hydrodynamic stiffness and damping
           'I',1,'Ig',1,...     % moment of inertia of flap and generator/powertrain
           'w',1, 'Fh',1, ...   % wave frequency and exciting force amplitude
           'GR',1,'T_spring',1, ...      % gear ratio, spring torque
           'tau_max_Nm',4, 'motor_max_rpm',3000,... % motor max torque and speed
           'T_s',.9,'T_d',.1,'b',.1);  % static and dynamic friction torques and viscous friction coefficient

% impedance matching
p.Kp = p.I * p.w^2 - p.Kh;
p.Bp = p.Bh;

% ode inputs
T = 2*pi/p.w;
y0 = [0;0];
yp0 = [0;0];
tspan = [0 7];

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
end

% plot solution
figure
plot(t,y, t,T_gen, t,T_fric, t,T_string, t,P)
xlabel('Time (s)')
legend('$\theta$','$\dot{\theta}$','$T_{gen}$','$T_{fric}$','$T_{string}$', ...
    'P','interpreter','latex','FontSize',14)

figure
xlabel('Time (s)')
ylabel('Integration Error')
plot(t,err(1,:),t,err(2,:))
legend('err_T (Nm)','err_v (m/s)')


function [value,isterminal,direction] = Events(t,y,yp,p)
%% event function for when net torque without friction abs(net torque) = p.T_s
% sol.te, sol.ye, and sol.ie store event information 
% states
th = y(1,:);
th_dot = y(2,:);
th_ddot = yp(2,:);

% effective gear ratio
alpha = 0; %% actual angle between string and flap from perpendicular be a function of theta
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
T_gen = min(max(T_gen/p.tau_max_Nm, -1), 1) * p.tau_max_Nm;
motor_rpm = GR_eff * th_dot * 60/(2*pi);
T_gen(abs(motor_rpm) > p.motor_max_rpm) = 0;
    
% friction
net_trq_without_fric = T_gen - T_string + p.T_spring;    
value = abs(net_trq_without_fric)-p.T_s; % if this is zero, then the motor sticks 
isterminal = 0;  % keep going 
direction = 0;   % The zero can be approached from either direction

end

% equation of motion
function [err,P,T_gen,T_fric,T_string] = dynamics(t,y,yp,p)
% states
th = y(1,:);
th_dot = y(2,:);

th_dot_next = yp(1,:);
th_ddot = yp(2,:);

% effective gear ratio
alpha = 0; %% actual angle between string and flap from perpendicular be a function of theta
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
T_gen = min(max(T_gen/p.tau_max_Nm, -1), 1) * p.tau_max_Nm;
motor_rpm = GR_eff * th_dot * 60/(2*pi);
T_gen(abs(motor_rpm) > p.motor_max_rpm) = 0;

% friction
zero_speed_idxs = ismembertol(th_dot,0);
net_trq_without_fric = T_gen - T_string + p.T_spring;
T_static = min(p.T_s,abs(net_trq_without_fric)) .* sign(net_trq_without_fric);
T_dynamic = p.T_d * sign(th_dot) + p.b * GR_eff * th_dot;
T_fric = zeros(size(T_gen));


 % linearize transition zone 
v_thresh = 0.05;
T_thresh = 0.05;
idx_pure_static = th_dot == 0; % & (abs(th_dot) <= T_thresh);
idx_pure_dynamic = (abs(th_dot) > v_thresh) | (0<abs(th_dot) & abs(th_dot) < v_thresh & abs(th_ddot)>0); 
idx_transition_to_d = abs(net_trq_without_fric) > T_static - T_thresh;
idx_transition_to_s = ((abs(th_dot) <= T_thresh) & (abs(th_dot) > 0)) & (abs(th_ddot)<0);
T_transition_to_d = T_dynamic+((T_static-T_dynamic)/v_thresh).*(T_static-net_trq_without_fric);%
T_transition_to_s = T_static + th_dot.*(T_dynamic-T_static)/T_thresh;


T_fric(idx_pure_static) = T_static(idx_pure_static);
T_fric(idx_pure_dynamic) = T_dynamic(idx_pure_dynamic);
T_fric(idx_transition_to_d) = T_transition_to_d(idx_transition_to_d);
T_fric(idx_transition_to_s) = T_transition_to_s(idx_transition_to_s);



%T_fric(zero_speed_idxs) = T_static(zero_speed_idxs);
%T_fric(~zero_speed_idxs) = T_dynamic(~zero_speed_idxs);

% unbalanced torque on drivetrain
T_unb = p.Ig * GR_eff * th_ddot - T_gen + T_fric + T_string - p.T_spring;

err = [T_unb; th_dot - th_dot_next];

% power

P = T_gen .* th_dot * GR_eff;

end
