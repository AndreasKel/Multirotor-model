%Author: AndreasKel
%----------------------------------------------------------------------------------------------------
%license:          MIT
%file name:        Transformation.m
%environment:      Matlab/Simulink
%functionality:    Rotation and transformation matrix transfers velocities from body to earth frame.  
%====================================================================================================
function [XDOT] = Transformation(E,V)      
    %Extract state vectors
    %earth frame
    X = E(1); %X-axis
    Y = E(2); %Y-axis
    Z = E(3); %Z-axis
    phi1 = E(4); %phi
    theta1 = E(5); %theta
    psi1 = E(6); %psi
    
    %body frame
    u = V(1); %u
    v = V(2); %v
    w = V(3); %w 
    p = V(4); %p
    q = V(5); %q
    r = V(6); %r

    Vdot = [u, v, w, p, q, r]';

    %Transformation matrices
    T_m = [1, sin(phi1)*tan(theta1), cos(phi1)*tan(theta1);
           0, cos(phi1), -sin(phi1);
           0, sin(phi1)/cos(theta1), cos(phi1)/cos(theta1)];
    R_m = [cos(psi1)*cos(theta1), cos(psi1)*sin(theta1)*sin(phi1)-sin(psi1)*cos(phi1), cos(psi1)*sin(theta1)*cos(phi1)+sin(psi1)*sin(phi1);
           sin(psi1)*cos(theta1), sin(psi1)*sin(theta1)*sin(phi1)+cos(psi1)*cos(phi1), sin(psi1)*sin(theta1)*cos(phi1)-cos(psi1)*sin(phi1);
           -sin(theta1), cos(theta1)*sin(phi1), cos(theta1)*cos(phi1)];
    J_m = [R_m, zeros(3);
           zeros(3), T_m];

    XDOT = J_m*Vdot;
end

