function plot_MultiCopter_Realtime(x,u)
% 

handles=getappdata(0,'myPlothandles');


euler_gain = 1;
x(abs(x)<1e-5)=0;


XYZ = x(4:6);

% Transformation of coordinates into Matlab frame
Tmb = [0  1  0; 
       1  0  0;
       0  0 -1];
XYZ = Tmb*XYZ;


phi   =	x(10)*euler_gain;
theta =	x(11)*euler_gain; 
psi	=   x(12);
sphi = sin(phi);   cphi = cos(phi);
sthe = sin(theta); cthe = cos(theta);
spsi = sin(psi); cpsi = cos(psi);
if abs(cthe)<=0.00001  % Hack to avoid singulariy at +/- pi/2
 cthe = 0.00001 * sign(cthe);
end
Tnb = [cthe * cpsi    sphi * sthe * cpsi - cphi * spsi    cphi * sthe * cpsi + sphi * spsi;
       cthe * spsi    sphi * sthe * spsi + cphi * cpsi    cphi * sthe * spsi - sphi * cpsi;
      -sthe           sphi * cthe                         cphi * cthe];
Wxr  = (Tmb*Tnb*handles.Ax')'; % rotation of x-axis


% Tilting of Rotors
faktor = 20;  % exaggeration faktor for drawing
% Define Rotation Matrices
delta1 = u(3)*faktor;
delta2 = u(4)*faktor;
sdel1 = sin(delta1); cdel1 = cos(delta1);
sdel2 = sin(delta2); cdel2 = cos(delta2);
RC1 = [1 0 0; 0 cdel1 -sdel1; 0 sdel1 cdel1];
RC2 = [1 0 0; 0 cdel2 -sdel2; 0  sdel2 cdel2];


nc = size(handles.circ,1);
C1r = (Tmb*Tnb*(ones(nc,1)*handles.Ax(4,:) + handles.circ * RC1')')';  % Rotor 1
C2r = (Tmb*Tnb*(ones(nc,1)*handles.Ax(1,:) + handles.circ * RC2')')';  % Rotor 1

% C1r  = (Tmb*Tnb*RC1*handles.C1')'; % rotation of rotor 1
% C2r  = (Tmb*Tnb*RC2*handles.C2')'; % rotation of rotor 2
 
Wxrt  = Wxr + ones(size(handles.Ax,1),1) * XYZ'; % translation of x-axis
set(handles.hx,'XData',Wxrt(:,1),'YData',Wxrt(:,2),'ZData',Wxrt(:,3))  

set(handles.hx1,'XData',Wxr(:,1),'YData',Wxr(:,2),'ZData',Wxr(:,3))  
set(handles.hC1,'XData',C1r(:,1),'YData',C1r(:,2),'ZData',C1r(:,3)) 
set(handles.hC2,'XData',C2r(:,1),'YData',C2r(:,2),'ZData',C2r(:,3)) 


drawnow limitrate
