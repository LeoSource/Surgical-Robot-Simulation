function [ end_Orientation ] = SlaveOrientation( jointPos )
%   此处显示有关此函数的摘要
%   此处显示详细说明

    
    global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R6t_orl
    global R01 R02 R03 R04 R05 R06 R0t
    
    RotZ     =  @(theta)([cos(theta) -sin(theta) 0
                          sin(theta)  cos(theta) 0
                              0           0      1]);

    %%%%%%%%%%%%%%各坐标系相对基坐标系的旋转矩阵%%%%%%%%%%%%%%%%
    R01     = R01_orl;                         %%坐标系1相对坐标系0的旋转矩阵
    R02     = R01*RotZ(jointPos(1))*R12_orl;   %%坐标系2相对坐标系0的旋转矩阵
    R03     = R02*RotZ(jointPos(2))*R23_orl;
    R04     = R03*RotZ(0)*R34_orl;             %%关节3位移动关节，未发生坐标系旋转
    R05     = R04*RotZ(jointPos(4))*R45_orl;
    R06     = R05*RotZ(jointPos(5))*R56_orl;
    R0t     = R06*RotZ(jointPos(6))*R6t_orl;

    end_Orientation = R0t;

end

