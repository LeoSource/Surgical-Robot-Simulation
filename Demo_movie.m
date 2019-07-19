clc
clear all

A0 = [0 0 0]';
A1 = [0 0 1]';
A2 = [0 1 0]';
d1 = 1;
d2 = 1;

for i=1:50
    
    A1(1) = cos(pi/2+i/100*pi)*d1;
    A1(2) = 0;
    A1(3) = sin(pi/2+i/100*pi)*d1;%%%%%%%%%%%A1ÈÆ×ÅyÖáÐý×ª
    
    
    plot3([A0(1),A1(1),A2(1)],[A0(2),A1(2),A2(2)],[A0(3),A1(3),A2(3)],'LineWidth',1.5);
    axis([-1,1,-1,1,0,1]);
    grid on;
    xlabel('x(m)');
    ylabel('y(m)');
    zlabel('z(m)');
    M(i) = getframe;
    
end
movie(M,2);
