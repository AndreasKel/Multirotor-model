%Author: AndreasKel
%---------------------------------------------------------------------------------------------
%license:          MIT
%file name:        cMultirotor.m
%environment:      Matlab/Simulink
%functionality:    Class to initialise a multirotor object.
%==============================================================================================

classdef cMultirotor<handle
    %CDRONE
    
    properties (Access = public)
        Mass
        Ixx
        Iyy
        Izz
        armLength
    end
    
    properties (Access = private)    
        air_den = 1.225;              %air density
        gravity = 9.81;
        propellerDiameter = 0.127;    %Diameter of each propeller
        rotorsTotal = 4;              %Total number of rotors on the drone
        tiltRotorAngle = 0;           %propulsion tilt angle
        qVelocityVector; 
        qPositionVector; 
        
        Cd = 0.3;
        areaProp;
        b_Co;                       % thrust Co - (air_den*Area_Prop *(0.1016)^2)/(8*pi*pi);        
        d_Co;                       % drag Co d = 2.98*10^-6; 
        Jr;                         % rotor inertia
    end
    
    methods
        function obj = cMultirotor(params)
            %cDRONE Construct an instance of this class
            obj.Mass = params('Mass');
            obj.armLength = params('armLength');
            obj.Ixx = params('Ixx');
            obj.Iyy = params('Iyy');
            obj.Izz = params('Izz');
            
            obj.areaProp = (pi*obj.propellerDiameter^2) /4;
            obj.b_Co = (obj.air_den*obj.areaProp *(0.1016)^2)/(8*pi*pi);                          %thrust Co        
            obj.d_Co = 0.5*(obj.propellerDiameter/2)^3 *obj.air_den*obj.areaProp*obj.Cd;          %drag Co
            obj.Jr = [0.00000365, 0.00000002, 0.00000014; -0.00000002, 0.00000404, 0; 0.00000014, 0, 0.00000391]; % rotor inertia matrix
        end
        
        function state = getVelocityState(obj)
            state = obj.qVelocityVector;
        end
        
        function state = getPositionState(obj)
            state = obj.qPositionVector;
        end
        
        function state = calcVelocityVectorDot(obj,V,U,X)
            %Extract state vector
            u = V(1); %u - linear velocity at x-axis body frame
            v = V(2); %v - linear velocity at y-axis body frame
            w = V(3); %w - linear velocity at z-axis body frame
            p = V(4); %p - angular velocity at x-axis body frame
            q = V(5); %q - angular velocity at y-axis body frame
            r = V(6); %r - angular velocity at z-axis body frame

            phi1 = X(4);   %phi angle - Φ   
            theta1 = X(5); %theta angle - Θ
            psi1 = X(6);   %psi angle - Ψ

            %Extract input vector (angular velocity of the rotors)
            omegaRotors = [U(1), U(2), U(3), U(4)];

            %Initialiasing variables
            wb = [p;q;r];

            vb = [u;v;w];
            
            %Drone constants 
            %----------------------
            I = [obj.Ixx, 0, 0; ...
                 0, obj.Iyy, 0; ...
                 0, 0, obj.Izz];                %Inertia matrix of the drone
            %----------------------

            prop_force = zeros(3,1);
            prop_torque = zeros(3,1);

            rotor_angle = 2*pi/obj.rotorsTotal; %Angle between two arms 
            if obj.tiltRotorAngle <= 0          %CW -> negative, CCW -> positive
                CR = eye(3);
            else
                CR = [-1,  0, 0;
                       0, -1, 0;
                       0,  0, 1];
            end
            for rotor = 0:(obj.rotorsTotal-1)
                tiltAngle = abs(obj.tiltRotorAngle);
                prop_pos = obj.armLength*[ sin(rotor* rotor_angle);
                           -cos(rotor* rotor_angle);
                           0];
                prop_orient = CR*[sin(rotor* rotor_angle)*sin(tiltAngle);
                                  -cos(rotor* rotor_angle)*sin(tiltAngle);
                                  cos(tiltAngle)];

                prop_force = prop_force + prop_orient * obj.b_Co * (omegaRotors(rotor+1))^2; 
                PR = (-1)^rotor;  %signum of propulsion rotation
                prop_torque = prop_torque + (cross(prop_pos,prop_orient)*obj.b_Co + PR*prop_orient*obj.d_Co)*(omegaRotors(rotor+1))^2;
            end 

            U = [prop_force;
                 prop_torque]; %6x1 matrix
             
            %----------------------------------------------------------------------
            %NO LONGER USED
            % U1 = b_Co*(w1^2 + w2^2 + w3^2 + w4^2);   %thrust force of propulsors
            % U2 = b_Co*L*(w4^2 - w2^2);               %roll torque
            % U3 = b_Co*L*(w3^2 - w1^2);               %pitch torque
            % U4 = d_Co*(-w1^2 + w2^2 - w3^2 + w4^2);  %yaw torque
            % Ub = [0; 0; U1; U2; U3; U4];             %Movement vector
            %-----------------------------------------------------------------------

            Mb = [obj.Mass*eye(3) zeros(3); zeros(3) I]; 
            Cb = [cross(wb,(obj.Mass*vb)); cross(wb,(I*wb))];      %coriolis-centripetal matrix
            Gb = [obj.Mass*obj.gravity*sin(theta1);                %gravitational vector
                  -obj.Mass*obj.gravity*cos(theta1)*sin(phi1);
                  -obj.Mass*obj.gravity*cos(theta1)*cos(phi1);
                  0;
                  0;
                  0];
            GyroscopicTorque= -1*((obj.Jr*(cross(wb,[0;0;1]))*-1*omegaRotors(1))+(obj.Jr*(cross(wb,[0;0;1]))*omegaRotors(2))+(obj.Jr*(cross(wb,[0;0;1]))*-omegaRotors(3))+(obj.Jr*(cross(wb,[0;0;1]))*omegaRotors(4)));
            Ob = [0;0;0;GyroscopicTorque]; %gyroscopic torque vector
            lambda = U + Ob + Gb; 

            %----------------------
            obj.qVelocityVector = V;                  %generalised velocity vector in body frame   
            obj.qPositionVector = X;                  %position vector in earth frame
            state = Mb\(-Cb + lambda);
        end
        
        function setRotorTiltAngle(obj,AngleOfRotors)
            %Sets the tilt angle the rotors have the main body with respect to XY plane 
            obj.tiltRotorAngle = AngleOfRotors;
        end
        
        function setRotorsTotal(obj, NumberofRotors)
            %Sets the total number of rotors on the drone
            obj.rotorsTotal =  NumberofRotors;
        end
        
        function setPropellerDiameter(obj,diameter)
            %Sets the diameter of the propellers
            obj.propellerDiameter =  diameter;
        end
        
        function setRotorInertia(obj,InertiaMatrix)
            %Sets the rotor inertia
            obj.Jr(1,1) =  InertiaMatrix(1,1);
            obj.Jr(1,2) =  InertiaMatrix(1,2);
            obj.Jr(1,3) =  InertiaMatrix(1,3);
            
            obj.Jr(2,1) =  InertiaMatrix(2,1);
            obj.Jr(2,2) =  InertiaMatrix(2,2);
            obj.Jr(2,3) =  InertiaMatrix(2,3);
            
            obj.Jr(3,1) =  InertiaMatrix(3,1);
            obj.Jr(3,2) =  InertiaMatrix(3,2);
            obj.Jr(3,3) =  InertiaMatrix(3,3);
        end
    end
end

