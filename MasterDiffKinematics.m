function [ jacobian ] = MasterDiffKinematics( input )
%   此处显示有关此函数的摘要
%   此处显示详细说明

    global Jv Jw
    global R01 R02 R03 R04 R05 R06 R07 R08 
    global P01 P02 P03 P04 P0t
    
    %%%%%%%%%%%%%%%%%运动学：雅克比矩阵求解%%%%%%%%%%%%%%%%%
    %%%%%%%目标坐标系At的速度/角速度由之前所有坐标系的运动叠加得到%%%%%%%%
    %%%%%%%wt = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8
    %%%%%%%vt = w1×P1t + w2×P2t + w3×P37 + w4×P4t 
    Ez            =  [0 0 1]';
    Jw(:,1)       =  R01*Ez;
    Jw(:,2)       =  R02*Ez - R03*Ez;
    Jw(:,3)       =  R04*Ez;
    Jw(:,4)       =  R05*Ez;
    Jw(:,5)       =  R06*Ez;
    Jw(:,6)       =  R07*Ez;
    Jw(:,7)       =  R08*Ez;

    Jv(:,1)       =  cross(Jw(:,1),(P0t - P01));
    Jv(:,2)       =  cross(R02*Ez,(P0t - P02)) - cross(R03*Ez,(P0t - P03));
    Jv(:,3)       =  cross(Jw(:,3),(P0t - P04));
    Jv(:,4)       =  zeros(3,1);
    Jv(:,5)       =  zeros(3,1);
    Jv(:,6)       =  zeros(3,1);
    Jv(:,7)       =  zeros(3,1);

    jacobian      =  [Jv ; Jw];

end

