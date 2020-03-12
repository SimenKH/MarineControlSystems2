function [tau,sDotNew] = fcn(eta,mode,StartPos,endPos,sDot,sDotDot,s) %s should be the final input for aestetic reasons
%tuning parameters:
if mode==1 %straight line
    %tuning
    mu=1; %mu>0
    Kp=diag([1,1,1]);
    K2=diag([1,1,1]);
    
    %Path Parametrisation
    etaD=zeros(3,1);
    etaDs=zeros(3,1);
    etaDss = [0;0;0];
    Trajectory = (endPos(:,1) - StartPos(:,1))*s + StartPos(:,1);
    DesiredHeading = atan((endPos(2)-StartPos(2))/(endPos(1)-StartPos(1)));
    etaD(1,1)= Trajectory(1);
    etaD(2,1)= Trajectory(2);
    etaD(3,1)= DesiredHeading;
    %HeadingS = 0;
    %HeadingSS = 0;
    etaDs(:,1) = (endPos(:,1) - StartPos(:,1));
    etaDs(:,2) = (endPos(:,2) - StartPos(:,2));
    xds=etaDs(:,1);
    yds=etaDs(:,2);
    xdss=0;
    ydss=0;
    
elseif mode==2 %ellipsoid
    %tuning
    mu=1; %mu>0, mu<0.1
    Kp=diag([1,1,1]);
    K2=diag([1,1,1]);
    %calculation
    etaD = zeros(3,1);
    Trajectory=zeros(3,1);
    etaDs = zeros(3,1);
    etaDss = zeros(3,1);

    c_x = (EtaDesired(1)-StartPos(1))/2;
    c_y = (EtaDesired(2)-StartPos(2))/2;
    r_x = sqrt(c_x^2 + c_y^2);
    r_y = 1/4 * r_x;
    Trajectory(1) = r_x*cos(2*pi*s);
    Trajectory(2) = r_y*sin(2*pi*s);
    DesiredHeading = arctan((r_y*sin(2*pi*s)+c_y)/(r_x*cos(2*pi*s)+c_x));
    etaDs(1) = -2*pi*r_x*sin(2*pi*s);
    etaDs(2) = 2*pi*r_y*cos(2*pi*s);
    etaDs(3) = ((-2*pi*r_x*sin(2*pi*s))*(-4*pi^2*r_y*sin(2*pi*s)) - ...
    (-4*pi^2*r_x*cos(2*pi*s)*-2*pi*r_y*cos(2*pi*s))) /...
    ((-2*pi*r_x*sin(2*pi*s))^2 + (2*pi*r_y*cos(2*pi*s))^2);
    
    etaDss(1) = -4*pi^2*cos(2*pi*s);
    
    etaDss(2) = -4*pi^2*sin(2*pi*s);
    
    etaDss(3) = 2*((-2*pi*r_x*sin(2*pi*s))*(-4*pi^2*r_y*sin(2*pi*s)) - ...
    (-4*pi^2*r_x*cos(2*pi*s)*-2*pi*r_y*cos(2*pi*s)))* ...
    (-2*pi*r_x*sin(2*pi*s)*-4*pi^2*r_x*cos(2*pi*s) + ...
    2*pi*r_y*cos(2*pi*s)*-4*pi^2*r_y*sin(2*pi*s)) /...
    ((-2*pi*r_x*sin(2*pi*s))^2 + (2*pi*r_y*cos(2*pi*s))^2)^2;
    etaD(1,1)= Trajectory(1);
    etaD(2,1)= Trajectory(2);
    etaD(3,1)= DesiredHeading;
    
    xds=etaDs(:,1);
    yds=etaDs(:,2);
    xdss=etaDss(:,1);
    ydss=etaDss(:,2);
end




%constants
R=[cos(eta(3,1)),-sin(eta(3,1)),0;sin(eta(3,1)),cos(eta(3,1)),0;0,0,1];
Rt=transpose(R);
M=[16,0,0;0,24,0.53;0,0.53,2.8];
D=[0.66,0,0;0,1.3,2.8;0,0,1.9];
Us=sDot*sqrt(xds^2+yds^2);

Uss=sDotDot*sqrt(xds^2+yds^2)+((sDot(xds*xdss+yds*ydss)/(sqrt(xds^2+yds^2))));

%actual controller eqs

z1=Rt*(eta-etaD);
alpha1=-Kp*z1+Rt*etaDs*Us;
alpha1s=Rt*(etaDss*Us+etaDs*Uss);
sigma1=-Kp*Rt; %should Rt be derivated here?

V1s=-transpose(z1)*Rt*etaDs;


z2=nu+alpha1;


tau=-z1+D*nu+M*alpha1s*Us+M*sigma1-K2*z2;
sDotNew=Us-(mu/abs(etaDs))*V1s;
end
