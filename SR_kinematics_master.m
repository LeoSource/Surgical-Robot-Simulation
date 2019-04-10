%%%%%%%%%%%%%%主手运动学模型%%%%%%%%%%%%
%%%%%%%%%%%%%左右手坐标系一致%%%%%%%%%%%
clc
clear
%%%%%%%%%%%%%%参数初始化%%%%%%%%%%%%%
left_length   =  [0.086, 0.0975, (0.28-0.00264), 0.107, (0.26-0.0079), 0.1396];
right_length  =  [0.086, 0.0975, (0.28+0.00186), 0.107, (-0.26+0.0048), 0.1396];
displayAngle  =  0*pi/180;

RotZ          =  @(theta)([ cos(theta) -sin(theta) 0
                            sin(theta)  cos(theta) 0
                                 0          0      1]);
                             
RotX          =  @(theta)([ 1       0           0
                            0   cos(theta)  -sin(theta)
                            0   sin(theta)   cos(theta)]);                             
                             
jointPos      =  [ 0 0 0 0 0 0 0];

%%%%%%%%%%%%%%%%各坐标系初始旋转矩阵%%%%%%%%%%%%%%%
R01_orl       =  eye(3);
R12_orl       =  [ 0 1 0; 0 0 1; 1 0 0];
R23_orl       =  eye(3);
R34_orl       =  [ 0 0 1; 1 0 0; 0 1 0];
R45_orl       =  eye(3);
R56_orl       =  [ 0 1 0; 0 0 1; 1 0 0];
R67_orl       =  [ 0 0 1; 1 0 0; 0 1 0];
R78_orl       =  [ 0 0 1; 1 0 0; 0 1 0];
R89_orl       =  [ 0 1 0; 0 0 1; 1 0 0];
R9t_orl       =  [ 0 0 1; -1 0 0; 0 -1 0];  %视野坐标系至A9坐标系的转换矩阵
%%%%%%%%初始视野坐标系对应人眼正下方看的视野%%%%%%%%%
%%%%%%%%遥操作：主手与从手的运动方向保持一致%%%%%%%%%
%%%%%%%%%%%%%视野坐标系：左——+X；下——+Y；靠近人——+Z%%%%%%%%%%%

%%%%%%%%%%%%%%%%%各坐标系原点位置%%%%%%%%%%%%%%%%%先以左手为例建立运动学模型
P01_orl       =  [ 0 0 -left_length(1)]';
P12_orl       =  [ 0 0 -left_length(2)]';
P23_orl       =  [ 0 left_length(3) 0]';
P34_orl       =  [ -left_length(4) 0 0]';
P45_orl       =  [ 0 left_length(5) left_length(6)]';%%A5/A6/A7/A8坐标系原点重合

%%%%%%%%%%%%%%%%%各坐标系相对基坐标系的旋转矩阵%%%%%%%%%%%%%%
display_orl   =  [ 0 -1 0; 0 0 -1; 1 0 0];
displayRot    =  RotX(-displayAngle)*display_orl;
R01           =  displayRot*R01_orl;%%考虑到视野坐标系的影响
R02           =  R01*RotZ(jointPos(1))*R12_orl;
R03           =  R02*RotZ(jointPos(2))*R23_orl;
R04           =  R03*RotZ(-jointPos(2))*R34_orl;
R05           =  R04*RotZ(jointPos(3))*R45_orl;
R06           =  R05*RotZ(jointPos(4))*R56_orl;
R07           =  R06*RotZ(jointPos(5))*R67_orl;
R08           =  R07*RotZ(jointPos(6))*R78_orl;
R09           =  R08*RotZ(jointPos(7))*R89_orl;
R0t           =  R09*R9t_orl;

%%%%%%%%%%%%%%%%%各坐标系原点相对基坐标系原点矢量在基坐标系下的表示%%%%%%%%%%%%%%
P01           =  displayRot*P01_orl;
P02           =  P01 + R01*RotZ(jointPos(1))*P12_orl;
P03           =  P02 + R02*RotZ(jointPos(2))*P23_orl;
P04           =  P03 + R03*RotZ(-jointPos(2))*P34_orl;
P0t           =  P04 + R04*RotZ(jointPos(3))*P45_orl;

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

jacobian      =  [Jv ; Jw]；
% Vt            =  jacobian*V;
% vt            =  Vt(1:3);
% wt            =  Vt(4:6);
