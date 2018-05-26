function xdot = DC_dyn(~,x,u,param)
% Calculates state derivatives xdot for dual copter model
% Author: Bernd Messnarz, Last Revision: 2017|03|30
% definition of states
% x =[u; v; w; x; y; z; p; q; r; phi; theta; psi]

  VKb = x(1:3);  % Kinematic velocity in body frame
  OKb = x(7:9);  % Angular velocity of the body in body frame


% Initialization
  nstates = length(x);  xdot  = zeros(nstates,1);

% DCM to transform wind into body axes              
  sphi = sin(x(10)); cphi = cos(x(10));
  sthe = sin(x(11)); cthe = cos(x(11));
  spsi = sin(x(12)); cpsi = cos(x(12));
  if abs(cthe)<=0.00001  % Hack to avoid singulariy at +/- pi/2
     cthe = 0.00001 * sign(cthe);
  end
  tthe = sthe/cthe; 
  Tbn = [cthe*cpsi,                cthe*spsi,                -sthe;
       sphi*sthe*cpsi-cphi*spsi, sphi*sthe*spsi+cphi*cpsi, sphi*cthe;
       cphi*sthe*cpsi+sphi*spsi, cphi*sthe*spsi-sphi*cpsi, cphi*cthe];
  Tnb = Tbn';
% Body drag force and moment (damping)
% no wind:
  VWn = [0;0;0]; OWn=[0;0;0];
  VAb = VKb- Tbn * VWn;   % for drag force
  OAb = OKb- Tbn * OWn;   % for drag moment
  % Drag force
  FDb = - param.cD * norm(VAb) .* VAb;
  % Drag moment
  MDb = - param.cmD * OAb;   

% Gravity in body frame
  gb = param.g *[-sthe; sphi*cthe; cphi*cthe];

  
  % DCM to tranform wind into body axes             
  sphi = sin(x(10)); cphi = cos(x(10));
  sthe = sin(x(11)); cthe = cos(x(11));
  spsi = sin(x(12)); cpsi = cos(x(12));
  if abs(cthe)<=0.00001  % Hack to avoid singulariy at +/- pi/2
     cthe = 0.00001 * sign(cthe);
  end
  Tbn = [cthe*cpsi,                cthe*spsi,                -sthe;
       sphi*sthe*cpsi-cphi*spsi, sphi*sthe*spsi+cphi*cpsi, sphi*cthe;
       cphi*sthe*cpsi+sphi*spsi, cphi*sthe*spsi-sphi*cpsi, cphi*cthe];
  VWb = Tbn * VWn;  
  OWb = Tbn * OWn;
    
% Velocities of rotors with respect to air
% VA = VK - VW  
  v1 = VKb+cross(OKb,param.r1) - (VWb+cross(OWb,param.r1));  
  v2 = VKb+cross(OKb,param.r2)-  (VWb+cross(OWb,param.r2));

  
  
% Matrix for forces:  Frotb = Fmat * un; 
  k1 = param.kT*(1+param.kv*v1(3));
  k2 = param.kT*(1+param.kv*v2(3));

% Thrust force of propellers  
  FT1 = k1*param.n2trim*u(1);
  FT2 = k2*param.n2trim*u(2);

% Moment of propellers  
  MT1 = param.kMT * FT1;
  MT2 = param.kMT * FT2;

  
  sd1 = sin(u(3)); cd1 = cos(u(3));    % sin(delta1), cos(delta1)
  sd2 = sin(u(4)); cd2 = cos(u(4));    % sin(delta2), cos(delta2)
  
% Total thrust force  
  FTb = [0;
        FT1*sd1+FT2*sd2;
       -FT1*cd1-FT2*cd2];
% Total thrust moment  
  MTb  = [FT1*sd1*param.H    + FT2*sd2*param.H;
          FT1*cd1*param.L/2  - FT2*cd2*param.L/2 + MT1*sd1 - MT2*sd2;
          FT1*sd1*param.L/2  - FT2*sd2*param.L/2 - MT1*cd1 + MT2*cd2];


% Equations of motion

% H_eulerdot_omega
  Hedom = [1  sphi*tthe   cphi*tthe;
           0  cphi       -sphi;
           0  sphi/cthe   cphi/cthe];       
             
% Force (drag, gravity and inertia)
  xdot(1:3)   = gb + (FTb+FDb)/param.mass -  cross(OKb, VKb); 
% Moment (drag, gravity and inertia)
  xdot(7:9)   = param.Ibi * ((MTb+MDb) -cross(OKb,param.Lsb) - cross(OKb, param.Ib * OKb));
% Position
  xdot(4:6)   = Tnb * VKb; 
% Attitude
  xdot(10:12) = Hedom * OKb;  
end