


%test values
tauRef=[1,1,0.5];
alpha=[pi*0.5,pi*0.5];
Balpha=[cos(alpha(1)),cos(alpha(2)),0;sin(alpha(1)),sin(alpha(2)),1;-lx*cos(alpha(1))-lx*sin(alpha(1)),-ly*cos(alpha(2))-lx*sin(alpha(2)),lx];
k1=5; %not final value just putting something here, not sure atm what should be here
k2=5; %as above
k3=5; %as above
K=diag(k1,k2,k3);


uStar=pinv(Balpha*K)*tauRef;

SchneiderThrusters=2;
BowThrusters=1;





function [alphaCmd,uCmd]= beta(uStar, alpha,SchneiderThrusters,BowThrusters)
%uStar from task
%alpha from task 
%Thrusters is an integer of how many thrusters there are
alphaCmd=alpha;
uCmd=uStar;

for i=1:ScheiderTrusters
    if uStar(i)<0
        uCmd(i)=-(uStar(i));
        if alpha(i)<0
            alphaCmd(i)=alpha(i)+pi;
        else
            alphaCmd(i)=alpha-pi;
        end
    else
        
    end
end
for i=ScheiderThrusters+1:BowThrusters
    uCmd(i)=uStar(i);
    
    
    
end


end









