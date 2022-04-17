%Drone Model
function [VDOT] = DRONE_MODEL(V,U,X)  

    drone1_params = containers.Map({'Mass','armLength','Ixx','Iyy','Izz'},...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468});

    quadcopter = cMultirotor(drone1_params);
    
    VDOT = quadcopter.calcVelocityVectorDot(V, U, X);
    
end

