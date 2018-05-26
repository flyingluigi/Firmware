% Define parameter structure for DC Model
% Author: Bernd Messnarz
% Revision History: 
% 2017|04|06: born
% 2017|04|10: error in param.r2 corrected
% Geometric data
  param.L  = 0.65;      % Distance between rotors, m
  param.H  = 0.1;       % Distance cg - rotor, m
  param.d  = 0.25;      % Rotor diameter, m
% Rotor positions   
  param.r1 = [ param.L/2;  0 ; -param.H];
  param.r2 = [-param.L/2;  0 ; -param.H]; 
  
% Aerodynamic rotor data  
  param.kT  = 7e-7;      % RPM^2 to thrust factor, N min^2   
  param.kMT = 0.0383;    % Thrust to moment conversion Factor, Nm/N  
  param.kv  = 0.05;      % Thrust reduction factor, s/m;
% Aerodynamic drag and damping of frame  
  param.cD  = 0.62; % = 1.24*1*0.1*1/2  (rho*CD*A*0.5) (=0.062)
  param.cmD = 0.01;  %   thumb times pi
  
% Mass, gravity and inertia
  param.g  = 9.80665;  % m/s^2
  param.mass = 10;     % Total mass, kg 
  Ixx  = 0.05; Iyy  = 0.8;  Izz  = 0.9;     % kgm^2
% Matrix of inertia
  param.Ib = [Ixx   0     0;
             0     Iyy   0;
             0     0     Izz];
% Inverse matrix of inertia
  param.Ibi = [1/Ixx  0       0;
           0      1/Iyy   0;
           0      0    1/Izz];  
  
% Additional, constant angular momentum due to spinning parts, kgm^2/s
  param.Lsb = [0;0;0];  % e.g. [0;0;0.1]

% Trim RPM^2
  param.n2trim = param.mass*param.g/(2*param.kT);
  
% maximum RPM^2 in terms of trim rpm^2
  param.unmax = 3;

% u = [n1^2 n2^2 delta1 delat2]
% un = [pitch roll yaw Tges]  
% un = T*u;   u = Ti*un
%            m1 m2 s1 s2
  param.T = [-1 1 0  0; % roll
              0 0 1  1; % pitch
              0 0 1 -1; % yaw
              1 1 0  0];% schub
          
  param.Ti = inv(param.T);
  
   Ti =      [1 0 0  1; % u1
             -1 0 0  1; % u2
              0 1 1  0; % s1
              0 1 -1  0];% s2
  