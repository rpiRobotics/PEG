%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Adds the column vector of body structs to the sim struct.  
% INPUTS:
%       sim  - A simulation struct created with Simulator().
%       body - A body struct or vector of body 
%               structs created with Body().  

function sim = sim_addBody( sim, body )

    % Allow body to contain multiple bodies, but require a vector.
    [r, c] = size(body);
    if min(r,c) > 1 
       error('Attempted to add a matrix of bodies to the simulator.'); 
    end
    
    % Force a row vector.
    if r > 1
        body = body'; 
    end
    
    % Update body information
    for b=1:length(body)
       % Force position vectors to be column vectors
       if size(body(b).pos,2) > 1, body(b).pos = body(b).pos'; end 
       body(b) = body_updatePosition(body(b),0);  
       body(b) = body_updateMesh(body(b));  
       body(b).bodyID = sim.num_bodies + b; 
    end

    % Update and return the sim struct. 
    sim.bodies = [ sim.bodies body ];
    sim.num_bodies = sim.num_bodies + length(body); 
end

