%Author: AndreasKel
%---------------------------------------------------
%license:          MIT
%file name:        InitialConditions.m
%environment:      Matlab/Simulink
%functionality:    Initial conditions of the model. 
%====================================================

clc, clear, close all
   
x0=[0;   %X
    0;   %Y
    0;   %Z
    0;   %phi
    0;   %theta
    0];  %psi

v0=[0;   %u
    0;   %v
    0;   %w
    0;   %p
    0;   %q
    0];  %r 


rpm_mot1 = 1000;
rpm_mot2 = 1000;
rpm_mot3 = 1000;
rpm_mot4 = 1000;


u_1 =  2*pi*rpm_mot1/60; 
u_2 =  2*pi*rpm_mot2/60; 
u_3 =  2*pi*rpm_mot3/60; 
u_4 =  2*pi*rpm_mot4/60;


%Battery parameters 
AmpsHour = 1.8;
Discharge = 80;      %C
L_ind = 1.17*10^-4;  %H
Kemf = 0.00255;      %Vs/rad
Kt = 0.00255;        %Nm/A
R = 0.117;           %Resistance

Jr_v = 6.5*10^-7;    %kg*m^2
b_m =2.415*10^-6;    %Nms

