% Animate DC Data
% Author: Bernd Messnarz
% Last Revision: 2017|06|01

euler_gain = 1;
simpace = 1;

nt=length(t);
tsim = t(end);
x(abs(x)<1e-5)=0;

UVW = x(:,1:3);
XYZ = x(:,4:6);
PQR = x(:,7:9)*180/pi;
PTP = x(:,10:12)*180/pi;

% Transformation of coordinates into Matlab frame
Tmb = [0  1  0; 
       1  0  0;
       0  0 -1];
XYZ = (Tmb*XYZ')';

UVWn = zeros(nt,3);
for i=1:nt
  phi   =	x(i,10);
  theta =	x(i,11); 
  psi	=   x(i,12);
  sphi = sin(phi);   cphi = cos(phi);
  sthe = sin(theta); cthe = cos(theta);
  spsi = sin(psi); cpsi = cos(psi);
  if abs(cthe)<=0.00001  % Hack to avoid singulariy at +/- pi/2
    cthe = 0.00001 * sign(cthe);
  end
  Tnb = [cthe * cpsi    sphi * sthe * cpsi - cphi * spsi    cphi * sthe * cpsi + sphi * spsi;
       cthe * spsi    sphi * sthe * spsi + cphi * cpsi    cphi * sthe * spsi - sphi * cpsi;
      -sthe           sphi * cthe                         cphi * cthe];
  UVWn(i,:) = (Tnb*UVW(i,:)')';
end

% figure
% plot(t,UVW,'Linewidth',2);
% xlabel('Time t (s)')
% ylabel('Velocities u,v,w (m/s)')
% legend('u','v','w');
% grid on

% figure
% plot(t,UVWn,'Linewidth',2);
% xlabel('Time t (s)')
% ylabel('Velocities un,vn,wn (m/s)')
% legend('un','vn','wn');
% grid on

%saveas(1,'UVW','pdf')

% figure
% plot(t,XYZ,'Linewidth',2);
% xlabel('Time t (s)')
% ylabel('Position pN, pE, pD (m)')
% legend('pN','pE','pD');
% grid on
%saveas(2,'XYZ','pdf')

% figure
% plot(t,PQR,'Linewidth',2);
% xlabel('Time t (s)')
% ylabel('Body Rates p, q, r (deg/s)')
% legend('p','q','r');
% grid on
%saveas(3,'PQR','pdf')

figure
plot(t,PTP,'Linewidth',2);
xlabel('Time t (s)')
ylabel('Euler Angles \phi, \theta, \psi (deg)')
legend('\phi','\theta','\psi');
grid on

figure
plot(XYZ(:,1),XYZ(:,2),'Linewidth',2);
xlabel('y (m)')
ylabel('x (m)')
title('Ground track');
grid on

figure
plot(t,XYZ(:,3),'Linewidth',2);
xlabel('Time t (s)')
ylabel('altitude h (m)')
title('Altitude');
grid on

htra=figure;
plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3),'Linewidth',2);
xlabel('y (m)')
ylabel('x (m)')
zlabel('h (m)')
title('Trajectory');
grid on
axis equal

%% Animation 
figure('units','normalized','outerposition',[0 0 1 1]) 

% Plot Trajectory (left plot)
subplot(1,2,1)
plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3),'Linewidth',2);
hold on
% Length of axes
Lx = param.L/2;
H  = param.H;
Rc = param.d/2;
Ax = [ -Lx    0     -H 
       -Lx    0      0        
        Lx    0      0
        Lx    0     -H];     

    
phic = (0:10:360)'*pi/180; nc=size(phic,1);   % Circle for rotors

Circ = [Rc*cos(phic)   Rc*sin(phic)     zeros(size(phic))]; % Rotor 1]
% Circle rotation matrix

delta1 = 0; 
delta2 = 0;
sdel1 = sin(delta1); cdel1 = cos(delta1);
sdel2 = sin(delta2); cdel2 = cos(delta2);

RC1 = [1 0 0; 0 cdel1 -sdel1; 0 sdel1 cdel1];
RC2 = [1 0 0; 0 cdel2 -sdel2; 0  sdel2 cdel2];

% Tilt rotors 1 3 5 7 
C1 = ones(nc,1)*Ax(4,:) + (RC1 * Circ')';  % Rotor 1
C2 = ones(nc,1)*Ax(1,:) + (RC2 * Circ')';  % Rotor 2
 
    
    
    
%phic = (0:10:360)'*pi/180;   % Circle for rotors
%Rc = param.d/2;
%Circ = [Rc*cos(phic)   Rc*sin(phic)     zeros(size(phic))]; 
%C1r = RC1*Circ';
%C2r = RC2*Circ';


%C1 = [Lx+Rc*cos(phic)   Rc*sin(phic)     -param.H*ones(size(phic))]; % Rotor 1
%C2 = [-Lx+Rc*cos(phic)  Rc*sin(phic)     -param.H*ones(size(phic))]; % Rotor 2

 
% Plot DC in Trajectory Plot
hx = plot3(Ax(:,1), Ax(:,2), Ax(:,3),'-o','MarkerSize',5,'Linewidth',2,'Color',[0 0.6 0]);
xlabel('y (m)'), ylabel('x (m)'), zlabel('h (m)')
title('Trajectory');
grid on, axis equal

% Plot DC in Attitude Plot
subplot(1,2,2);
hx1 = plot3(Ax(:,1), Ax(:,2), Ax(:,3),'-o','MarkerSize',10,'Linewidth',5,'Color',[0 0.6 0]);
hold on
hC1 = plot3(C1(:,1), C1(:,2), C1(:,3),'Linewidth',5,'Color',[0 0.9 0]);
hC2 = plot3(C2(:,1), C2(:,2), C2(:,3),'Linewidth',5,'Color',[0.6 0 0]);
xlabel('y (m)'), ylabel('x (m)'), zlabel('h (m)')
title('Attitude');
grid on, axis(1.5*[-Lx Lx -Lx Lx -Lx Lx]), axis manual
hold on

% Do animation

for i=1:nt
   if i==1
       disp('Press any key to start animation ...')
      % pause
       disp('Running animation ...')
       tic
  end  
  phi   =	x(i,10)*euler_gain;
  theta =	x(i,11)*euler_gain; 
  psi	=   x(i,12);
  sphi = sin(phi);   cphi = cos(phi);
  sthe = sin(theta); cthe = cos(theta);
  spsi = sin(psi); cpsi = cos(psi);
  if abs(cthe)<=0.00001  % Hack to avoid singulariy at +/- pi/2
    cthe = 0.00001 * sign(cthe);
  end
   Tnb = [cthe * cpsi    sphi * sthe * cpsi - cphi * spsi    cphi * sthe * cpsi + sphi * spsi;
         cthe * spsi    sphi * sthe * spsi + cphi * cpsi    cphi * sthe * spsi - sphi * cpsi;
        -sthe           sphi * cthe                         cphi * cthe];
   Wxr  = (Tmb*Tnb*Ax')'; % rotation of x-axis
   C1r  = (Tmb*Tnb*C1')'; % rotation of rotor 1
   C2r  = (Tmb*Tnb*C2')'; % rotation of rotor 2
 
   Wxrt  = Wxr + ones(size(Ax,1),1) * [XYZ(i,1),XYZ(i,2), XYZ(i,3)]; % translation of x-axis
   set(hx,'XData',Wxrt(:,1),'YData',Wxrt(:,2),'ZData',Wxrt(:,3))  

   set(hx1,'XData',Wxr(:,1),'YData',Wxr(:,2),'ZData',Wxr(:,3))  
   set(hC1,'XData',C1r(:,1),'YData',C1r(:,2),'ZData',C1r(:,3)) 
   set(hC2,'XData',C2r(:,1),'YData',C2r(:,2),'ZData',C2r(:,3)) 
  
   while toc*simpace - t(i)<0
    % waiting loop
   end
   drawnow limitrate
end
disp('Animation finished');
