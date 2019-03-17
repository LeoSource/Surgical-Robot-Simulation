%%%%%%%%%%%%%%从手运动学模型%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
%%%%%%%%%%%%%%参数初始化%%%%%%%%%%%%%
theta1   =  30/180*pi;
length   =  [0.16 0.7369 0 0.338 0 0.009 0];   %%各坐标系原点初始间距：坐标系0原点到坐标系1原点向量在坐标系0的表示，后续以此类推

RotZ     =  @(theta)([cos(theta) -sin(theta) 0
                      sin(theta)  cos(theta) 0
                          0           0      1]);
                      
                      
jointPos =  [10 30 0.1*pi/180 100 40 50]/180*pi;
%%%%%%%%%%%%%%%%%%%各坐标系初始旋转矩阵%%%%%%%%%%%%%%%%
R01_orl  =  [       0     1      0
              sin(theta1) 0 cos(theta1)
              cos(theta1) 0 -sin(theta1)];
R12_orl  =  [ sin(theta1) cos(theta1)  0
                  0             0      1
              cos(theta1) -sin(theta1) 0];
R23_orl  =  [0 1 0
             0 0 1
             1 0 0];
R34_orl  =  [1 0 0
             0 1 0
             0 0 1];
R45_orl  =  [0 0 1
             1 0 0
             0 1 0];
R56_orl  =  [0 0 1
             1 0 0
             0 1 0];
R6t_orl  =  [0 0 1
             1 0 0
             0 1 0];

%%%%%%%%%%%%%%%%%%%坐标系原点位置%%%%%%%%%%%%%%%%%%%%%%
P01_orl  =  [0 0 -length(1)]';  %%坐标系1的原点在坐标系0中的表示
P12_orl  =  [0 0 length(2)]';
P23_orl  =  [0 0 0]';
P34_orl  =  [0 0 -length(4)]';
P45_orl  =  [0 0 0]';
P56_orl  =  [0 -length(6) 0]';
P6t_orl  =  [0 0 0]';

%%%%%%%%%%%%%%各坐标系相对基坐标系的旋转矩阵%%%%%%%%%%%%%%%%
R01     = R01_orl;                         %%坐标系1相对坐标系0的旋转矩阵
R02     = R01*RotZ(jointPos(1))*R12_orl;   %%坐标系2相对坐标系0的旋转矩阵
R03     = R02*RotZ(jointPos(2))*R23_orl;
R04     = R03*RotZ(0)*R34_orl;             %%关节3位移动关节，未发生坐标系旋转
R05     = R04*RotZ(jointPos(4))*R45_orl;
R06     = R05*RotZ(jointPos(5))*R56_orl;
R0t     = R06*RotZ(jointPos(6))*R6t_orl;

%%%%%%%%%%%%%各坐标系原点相对基坐标系原点矢量在基坐标系下表示%%%%%%%%%%%%%%%
P01    = P01_orl;
P02    = P01 + R01*RotZ(jointPos(1))*P12_orl;
P03    = P02 + R02*RotZ(jointPos(2))*P23_orl;
P04    = P03 + R03*([0 ; 0; jointPos(3)] + P34_orl);
P05    = P04 + R04*RotZ(jointPos(4))*P45_orl;
P06    = P05 + R05*RotZ(jointPos(5))*P56_orl;
P0t    = P06 + R06*RotZ(jointPos(6))*P6t_orl;

%%%%%%%%%%%%%%%%%运动学：雅克比矩阵求解%%%%%%%%%%%%%%%%%
