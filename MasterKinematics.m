function [ CartRot, CartPos ] = MasterKinematics( jointPos,displayAngle )
% 此处显示有关此函数的摘要
%   此处显示详细说明
    
    global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R67_orl R78_orl R89_orl R9t_orl display_orl
    global P01_orl P12_orl P23_orl P34_orl P45_orl 
    global P01 P02 P03 P04 P0t
    global R01 R02 R03 R04 R05 R06 R07 R08 R09 R0t displayRot

    RotZ          =  @(theta)([ cos(theta) -sin(theta) 0
                            sin(theta)  cos(theta) 0
                                 0          0      1]);
                             
    RotX          =  @(theta)([ 1       0           0
                                0   cos(theta)  -sin(theta)
                                0   sin(theta)   cos(theta)]);      
    
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
    
    CartRot = R0t;
    CartPos = P0t;

end

