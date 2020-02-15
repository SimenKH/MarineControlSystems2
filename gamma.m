function [uCmd,alphaCmd]=gamma(uStar)
% param u_decomposed = [u1_x, u1_y, u2_x, u2_y, u3]
uCmd=[0,0,0];
alphaCmd=[0,0];
% TODO: limit output u to [0,1]?
u1=[0,0];
u2=u1;

u1(1) = uStar(1);%u1(1) is x, u1(2) is y
u1(2) = uStar(2);
u2(1) = uStar(3);
u2(2)= uStar(4);



alphaCmd(1) = atan2(u1(2), u1(1));
alphaCmd(2) = atan2(u2(2), u2(1));

uCmd(1) = norm(u1);
uCmd(2) = norm(u2);
uCmd(3) = uStar(5);
end