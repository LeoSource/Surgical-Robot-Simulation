function [ jacobian ] = SlaveDiffKinematics( input )
%%%%%%%%%%%%%%%%%运动学：雅克比矩阵求解%%%%%%%%%%%%%%%%%
%%%%%%%目标坐标系At的速度/角速度由之前的A1~A6所有坐标系的运动叠加得到%%%%%%%%
%%%%%%%wt = w1 + w2 + w3 + w4 + w5 + w6
%%%%%%%vt = w1×P1t + w2×P2t + v3 + w4×P4t + w5×P5t + w6×P6t 
   
    global R01 R02 R03 R04 R05 R06
    global P01 P02 P03 P04 P05 P06 P0t
    global Jw Jv
    
    Ez       = [0 0 1]';
    Jw(:,1)  = R01*Ez;
    Jw(:,2)  = R02*Ez;
    Jw(:,3)  = zeros(3,1);%关节3为移动关节
    Jw(:,4)  = R04*Ez;
    Jw(:,5)  = R05*Ez;
    Jw(:,6)  = R06*Ez;

    Jv(:,1)  = cross(Jw(:,1),(P0t - P01));
    Jv(:,2)  = cross(Jw(:,2),(P0t - P02));
    Jv(:,3)  = R03*Ez;%关节3为移动关节
    Jv(:,4)  = cross(Jw(:,4),(P0t - P04));
    Jv(:,5)  = cross(Jw(:,5),(P0t - P05));
    Jv(:,6)  = cross(Jw(:,6),(P0t - P06));

    jacobian = [Jv ; Jw];


end

