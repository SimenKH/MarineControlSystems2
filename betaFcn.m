
function [alphaCmd,uCmd]= betaFcn(uStar, alpha,Thrusters,BowThrusters)
%uStar from task
%alpha from task 
%Thrusters is an integer of how many thrusters there are
alphaCmd=alpha;
uCmd=uStar;

for i=1:Thrusters
    if uStar(i)<0
        uCmd(i)=-(uStar(i));
        if alpha(i)<0
            alphaCmd(i)=alpha(i)+pi;
        else
            alphaCmd(i)=alpha(i)-pi;
        end
    else
        
    end
end
for i=Thrusters+1:BowThrusters
    uCmd(i)=uStar(i);
    
    
    
end


end





