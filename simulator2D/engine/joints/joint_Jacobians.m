%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function [G1c, G2c] = joint_Jacobians( sim, j )

    Jnt = sim.joints(j); 
    body1 = sim.bodies(Jnt.body1id);
    body2 = sim.bodies(Jnt.body2id);

    p1 = Jnt.P1; 
    x1 = Jnt.X1 - p1;
    y1 = Jnt.Y1 - p1;
    r1 = p1 - body1.pos;

    p2 = Jnt.P2;
%     x2 = Jnt.X2 - p2;
%     y2 = Jnt.Y2 - p2;
%     z2 = Jnt.Z2 - p2;
    r2 = p2 - body2.pos; 

    zrs = zeros(3,1); 

    % Not compuationally efficient, but let's write the whole thing
    if ~body1.dynamic
       G1c = zeros(3);
    else
       G1c = [      x1               y1      
              cross2d(r1,x1)   cross2d(r1,y1)  ];
    end
    if ~body2.dynamic
       G2c = zeros(3);
    else
       G2c = [     -x1              -y1          
              cross2d(r2,-x1)   cross2d(r2,-y1)   ];
    end

    % Apply mask
    G1c = G1c(:,Jnt.mask);
    G2c = G2c(:,Jnt.mask); 

end


