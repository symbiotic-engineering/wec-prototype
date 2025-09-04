%% finding required mass of float to maintain desired draft
clear all
clc

g = 9.81;                               % gravitational constant [m/s^2]
rho_w = 1000;                           % density of water [kg/m^3]
r = linspace(2,15,14).*(0.0254/2);      % radius of float [m] linspace(2,15,14) actual value = 11.5"
l = linspace(1,5,14).*0.0254;           % length of float [m] linspace(1,8,14) actual value = 3.5"
m_rb = 0.337;                           % mass of rack + bridge [kg]
numRow = length(l);
numCol = length(r);
m_PA = zeros(numRow,numCol);
K = zeros(numRow,numCol);
A_33 = zeros(numRow,numCol);
m_float = zeros(numRow,numCol);
w = zeros(numRow,numCol);

for i = 1:1:numRow
    for j = 1:1:numCol
        m_PA(i,j) = (3/4)*l(i)*rho_w*pi*(r(j)^2);          % mass of PA [kg] (if draft is 3/4 of length)
        %A = 2*pi*r(j)*(l(i)/2) + 2*pi*r(j)^2;             % wetted surface area [m^2] (why was i using wetted surface area????? it works a lot better...)
        A = pi*r(j)^2;                                     % waterplane area
        K(i,j) = rho_w*g.*A;                               % PA stiffness [kg/s^2]
        A_33(i,j) = rho_w * (pi * r(j)^2) * (3*l(i)/4);    % added mass approx in heave (was originally l/2 for some reason?
        if m_PA(i,j) >= m_rb
            m_float(i,j) = m_PA(i,j) - m_rb;               % Mass of float
        else
            m_float(i,j) = NaN; % Set to NaN if buoyant mass is insufficient
        end
        w(i,j) = sqrt(K(i,j)./(m_PA(i,j) + A_33(i,j)));             % natural frequency
    end
end
disp(m_float)
T = (2*pi)./w;  
disp(T)
disp(A_33)

l_vec = linspace(0.0245,0.0889,10);
r_vec = linspace(0.0245,0.2921/2,10);
d_max = ones(size(l_vec)).*0.2921;         % maximum diameter [m]
l_max = ones(size(r_vec)).*0.0889;         % maximum length [m]

figure(1)
hold on
contourf(2.*r,l,m_float)
rectangle('Position', [2*r_vec(1),l_vec(1),d_max(1)-2*r_vec(1),l_max(1)-l_vec(1)], ...
                'FaceColor', [.0, .62, 0.451, 0.5], ...
                'EdgeColor', [.0, .62, 0.451, 0.5]);
plot(d_max,l_vec,'Color','#009E73','LineWidth',5)
plot(2.*r_vec,l_max,'Color','#009E73','LineWidth',5)
plot(0.2921,0.0889,"p",'MarkerEdgeColor','#000000','MarkerFaceColor','#009E73','MarkerSize',20,'LineWidth',2)
legend('','Achievable Range','','Chosen Dims','Location','northoutside')
xlabel('Diameter [m]','FontSize',16)
ylabel('Length [m]','FontSize',16)
ax = gca; 
ax.FontSize = 16;
c = colorbar;
c.Label.String = 'Mass of Float [kg]';
c.Label.FontSize = 16;
hold off

T = (2*pi)./w;                          % natural period
figure(2)
hold on
contourf(2.*r,l,T)
rectangle('Position', [2*r_vec(1),l_vec(1),d_max(1)-2*r_vec(1),l_max(1)-l_vec(1)], ...
                'FaceColor', [.0, .62, 0.451, 0.5], ...
                'EdgeColor', [.0, .62, 0.451, 0.5]);
plot(d_max,l_vec,'Color','#009E73','LineWidth',5)
plot(2.*r_vec,l_max,'Color','#009E73','LineWidth',5)
plot(0.2921,0.0889,"p",'MarkerEdgeColor','#000000','MarkerFaceColor','#009E73','MarkerSize',20,'LineWidth',2)
legend('','Achievable Range','','Chosen Dims','Location','northoutside')
xlabel('Diameter [m]','FontSize',16)
ylabel('Length [m]','FontSize',16)
ax = gca; 
ax.FontSize = 16;
c = colorbar;
c.Label.String = 'Natural Period [s]';
c.Label.FontSize = 16;
hold off

% %% solving for resonance
% clear all
% clc
% 
% g = 9.81;               % gravitational constant [m/s^2]
% rho_w = 1000;           % density of water [kg/m^3]
% r = 11.5*(0.0254/2);    % radius of float [m]
% l = 2*0.0254;           % length of float [m]
% m_rb = 0.337;           % mass of rack + bridge [kg]
% m_PA = linspace(1,50,20);
% numRow = length(m_PA);
% K = zeros(numRow,1);
% A_33 = zeros(numRow,1);
% COB = zeros(numRow,1);
% w = zeros(numRow,1);
% 
% for i = 1:1:numRow
%     COB(i) = m_PA(i)./(2*pi*r^2*rho_w);
%     h = 2*COB(i);
%     A = 2*pi*r*h + 2*pi*r^2;       % wetted surface area [m^2]
%     K(i) = rho_w*g.*A;                % PA stiffness [kg/s^2]
%     A_33(i) = pi*rho_w*(pi*r^2)*h;    % added mass approx in heave
%     w(i) = sqrt(K(i)/(m_PA(i) + A_33(i)));        % natural frequency
% end
% 
% m_float = m_PA - m_rb;
% T = (2*pi)./w;                     % natural period
% f = 1./T;
% 
% figure(3)
% plot(m_float,f)
% xlabel('mass of float [kg]')
% ylabel('natural frequency [Hz]')




