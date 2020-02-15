%task 3.1/3.4
function [uCmd,alphaCmd]=Task3()
lx=0.055;
ly=0.4574;
K1 = 1.03;
K2 = K1;
K3 = 2.629;
K = diag([K1, K1, K2, K2, K3]);
tauRef=[1;1;0.5];
alpha=[pi*0.5,pi*0.5];

Balpha=[1,0,1,0,0;0,1,0,1,1;ly,-lx,-ly,-lx,lx];

uStar=pinv(Balpha*K)*tauRef;

Thrusters=2;
BowThrusters=1;

%test values

[alphaCmd,uCmd]=gamma(uStar, alpha,Thrusters,BowThrusters);
end
