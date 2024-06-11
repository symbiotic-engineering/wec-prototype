clear all; close all; clc

%% Volumes
rtube = 0.01; %m  %2 cm diameter rods
ltube= 0.45; %m
rho = 600;%kg/m^3
vtube = ltube*rtube^2*pi;
vstrut = 92377e-9       ; %m^3 this is from Robert's calc but it is prob safe to assume

%% masses
mtube = vtube*rho; %kg
mstrut = 0.251; %kg
mpulley = 0.364*(2/3); %kg
mshaft = 2.111; %kg

%% Centers (individual)
c0 = 0.02; % spacing between the bottom of the tank and the start of the first tube.
t1 = c0+rtube;
t2 = t1+2*rtube;
t3 = t2+2*rtube;
t4 = t3+rtube*2;
t5 = t4+rtube*2;
t6 = t5+rtube*2;
t7 = t6+rtube*2;
t8 = t7+rtube*2;
t9 = t8+rtube*2;
t10 = t9+rtube*2;
t11 = t10+rtube*2;
t12 = t11+rtube*2;
ctube = [t12;t11;t10;t9;t8;t7;t6;t5;t4;t3;t2;t1]' ; %m
cstrut = t6; %m

%% Radii and Moments of Inertia
Iztube = (mtube/2)*(rtube^2); %kgm^2 - Moment of inertia of the pipe about it's central axis
Izstrut = 3.22*10^-5; %kgm^2 - Moment of inertia of the vertical struts (from Solid Edge)

rpulley = 0.0254    ; %m
Izpulley = (mpulley/2)*(rpulley^2); %kgm^2 - Moment of inertia of the pulley (estimate, taken as a solid cylinder)

rshaft = .0254/2; %m
Izshaft = (mshaft/2)*(rshaft^2); %kgm^2 - moment of inertia of the rotary shaft

%% Totals and Body Centers
vtotf = 3*(vstrut) + 12*vtube       ; %m^3 - total volume of the flap
mflap = 3*(mstrut) + 12*(mtube); %kg - total mass of the flap
mtot = mflap+mpulley+mshaft; %kg - total mass of the flap, pulley, and shaft

covf = (sum(vtube.*ctube) + 3*vstrut*cstrut)/vtotf; %m - center of volume of the flap
comf = (sum(mtube.*ctube) + 3*mstrut*cstrut)/mflap; %m - center of mass of the flap

Itubeoffset = Iztube+(mtube.*ctube.^2);% kgm^2 - parallel axis theorem
Imodel = sum(Itubeoffset)+Izshaft+Izpulley+3*(Izstrut+mstrut*cstrut^2);   % kgm^2 - calculated moment of inertia
Imref = mflap*comf;    % kgm^2 - calculated reference moment of inertia for the system - assuming flap as a point mass at the center of mass of the current flap
I_full_scale = 1.86*10^6; %kgm^2 from WECSIM
lambda = 36; %scaling factor
Iscale = I_full_scale/(lambda^5) %literature moment of inertia from WECSIM and scaling


rho_water = 997; %kg/m^3
g = 9.81; %m/s^2
Kmodel = rho_water*g*vtotf*covf - 9.81*mflap*comf      % Stiffness of the flap
kscale = 0.653                                  % Theoretical stiffness of the flap. Again, numbers come from CAD of full scale system
