% Define plothandles structure for live data display (Animation) of 
% the Model
% Author: Arno Fallast
% Revision History: 
% 2017|06|01: initial release

% plothandles
handles.hf=figure(10);
set(handles.hf,'units','normalized','outerposition',[0 0 1 1]) 
handles.ha(1)=subplot(1,2,1);


% Length of axes
Lx = param.L/2;
H  = param.H;
Rc = param.d/2;
Ax = [ -Lx    0     -H 
       -Lx    0      0        
        Lx    0      0
        Lx    0     -H];     
    
phic = (0:10:360)'*pi/180; nc=size(phic,1);   % Circle for rotors
circ = [Rc*cos(phic)   Rc*sin(phic)     zeros(size(phic))]; % Rotor 1]
% Circle rotation matrix

delta1 = 0; 
delta2 = 0;
sdel1 = sin(delta1); cdel1 = cos(delta1);
sdel2 = sin(delta2); cdel2 = cos(delta2);

RC1 = [1 0 0; 0 cdel1 -sdel1; 0 sdel1 cdel1];
RC2 = [1 0 0; 0 cdel2 -sdel2; 0  sdel2 cdel2];

% Tilt rotors 1 and 2 
C1 = ones(nc,1)*Ax(4,:) + (RC1 * circ')';  % Rotor 1
C2 = ones(nc,1)*Ax(1,:) + (RC2 * circ')';  % Rotor 2
 
   
    
%phic = (0:10:360)'*pi/180;   % Circle for rotors
%Rc = param.d/2;
%Circ = [Rc*cos(phic)   Rc*sin(phic)     zeros(size(phic))]; 
%C1r = RC1*Circ';
%C2r = RC2*Circ';


%C1 = [Lx+Rc*cos(phic)   Rc*sin(phic)     -param.H*ones(size(phic))]; % Rotor 1
%C2 = [-Lx+Rc*cos(phic)  Rc*sin(phic)     -param.H*ones(size(phic))]; % Rotor 2

 
% Plot DC in Trajectory Plot
handles.hx = plot3(handles.ha(1),Ax(:,1), Ax(:,2), Ax(:,3),'-o','MarkerSize',5,'Linewidth',2,'Color',[0 0.6 0]);
hold(handles.ha(1),'on')
xlabel('y (m)'), ylabel('x (m)'), zlabel('h (m)')
title('Trajectory');
grid on, axis equal
axis([-100 100 -100 100 -100 100])


handles.ha(2)=subplot(1,2,2);

% Plot DC in Attitude Plot
handles.hx1 = plot3(handles.ha(2),Ax(:,1), Ax(:,2), Ax(:,3),'-o','MarkerSize',10,'Linewidth',5,'Color',[0 0.6 0]);
hold(handles.ha(2),'on')
handles.hC1 = plot3(handles.ha(2),C1(:,1), C1(:,2), C1(:,3),'Linewidth',5,'Color',[0 0.9 0]);
handles.hC2 = plot3(handles.ha(2),C2(:,1), C2(:,2), C2(:,3),'Linewidth',5,'Color',[0.6 0 0]);
xlabel('y (m)'), ylabel('x (m)'), zlabel('h (m)')
title('Attitude')
grid on
axis(1.8*[-Lx Lx -Lx Lx -Lx Lx])
axis manual


handles.Ax = Ax;
handles.C1 = C1;
handles.C2 = C2;
handles.circ = circ;

%store the plothandles data globally
setappdata(0,'myPlothandles',handles)



