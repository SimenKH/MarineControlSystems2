function [alphaCmd,uCmd]=etaToUCmdAlphaCmdUsingBeta(eta)
lx=0.055;
ly=0.4574;
K1 = 1.03;
K2 = K1;
K3 = 2.629;

alpha=eta(something); %need to pick out correct values
tauRef=eta(somethingElse); %need to pick out correct values

K = diag([K1, K2, K3]);

Balpha=[cos(alpha(1)),cos(alpha(2)),0;sin(alpha(1)),sin(alpha(2)),1;-lx*cos(alpha(1))-lx*sin(alpha(1)),-ly*cos(alpha(2))-lx*sin(alpha(2)),lx];

uStar=pinv(Balpha*K)*tauRef;

Thrusters=2;
BowThrusters=1;

%test values

[alphaCmd,uCmd]=beta(uStar, alpha,Thrusters,BowThrusters);
end