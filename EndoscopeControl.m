%%%%%内窥镜控制方案仿真

clear 
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R6t_orl
global P01_orl P12_orl P23_orl P34_orl P45_orl P56_orl P6t_orl
global P01 P02 P03 P04 P05 P06 P0t
global R01 R02 R03 R04 R05 R06
global Jw Jv

%{
jointPos = [ 10 -20 0.2*180/pi 0 0 0 ]*pi/180';
endoscopeAngle = 30;
endo_Scale = 1;

nothing = SlaveInit(0);
endo_Rot = SlaveOrientation(jointPos);
endo_Pos = SlavePosition(jointPos);
endo_Rot = endo_Rot*rotx(endoscopeAngle);
endo_Jaco = SlaveDiffKinematics(0);

masterVel = [ 0.1 0.0 0.0 0 0 0 ]';
endo_LinearVel = endo_Rot * masterVel(1:3) * endo_Scale;
endo_AngularVel = endo_Rot * masterVel(4:6);%%内窥镜控制时不考虑主手移动产生的角速度
endo_RefVel = [endo_LinearVel;endo_AngularVel];

w_theta = 0.0;
tmp_AngularVel = endo_Rot * [ 0 0 w_theta]';
endo_RefVel = endo_RefVel + [ 0; 0; 0; tmp_AngularVel];
endo_jointVel = pinv(endo_Jaco) * endo_RefVel;

%%%%%%%%%%%%%%Separate Position and Rotation%%%%%%%%%%%%%
endo_jointVel(5) = 0;
endo_jointVel(6) = 0;  %%内窥镜臂的5/6关节无法运动，自由度缺失
endo_RefLineVel = endo_RefVel(1:3);
endo_RefAngVel = endo_RefVel(4:6);
Jv1 = endo_Jaco(1:3,1:3);
Jv2 = endo_Jaco(1:3,4:6);
Jw1 = endo_Jaco(4:6,1:3);
Jw2 = endo_Jaco(4:6,4:6);
%}

slave_setjointPos = [ 10 0 0 0 ];            %从手调整臂关节角
slave_shoulderjointPos = 20;                 %从手子悬吊关节角
endo_setjoitPos = [ 10 0 0 0 ];              %内窥镜调整臂关节角
endo_shoulderjointPos = 20;                  %内窥镜子悬吊关节角
endo_jointPos = [ 0 0 0 0 0 0];
s_Rot = [1 0 0
         0 1 0
         0 0 1];                             %Ti_Rot_T
m_Rot = MasterFollowRotation( slave_setjointPos,slave_shoulderjointPos,endo_setjoitPos,endo_shoulderjointPos,endo_jointPos,s_Rot )
