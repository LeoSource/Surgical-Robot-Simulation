%%%%%%%%%%%%%%主手运动学模型%%%%%%%%%%%%
%%%%%%%%%%%%%左右手坐标系一致%%%%%%%%%%%
clc
clear
%%%%%%%%%%%%%%参数初始化%%%%%%%%%%%%%
left_length   =  [0.086, 0.0975, (0.28-0.00264), 0.107, (0.26-0.0079), 0.1396];
right_length  =  [0.086, 0.0975, (0.28+0.00186), 0.107, (-0.26+0.0048), 0.1396];
displayAngle  =  90*pi/180;

RotZ          =  @(theta)([ cos(theta) -sin(theta) 0
                            sin(theta)  cos(theta) 0
                                 0          0      1]);
                             
RotX          =  @(theta)([ 1       0           0
                            0   cos(theta)  -sin(theta)
                            0   sin(theta)   cos(theta)]);                             
                             
jointPos      =  [ 0 0 0 0 0 0 0];
cmdJointPosS  =  [ 0 0 0 0 0 0 0];
cmdJointPosB  =  [ 0 0 0 0 0 0 0];

%%%%%%%%%%%%%%%%%%%%笛卡尔工作空间求解%%%%%%%%%%%%%%%%%%
D2R = pi/180;
R2D = 180/pi;
joint1Interval = 2*D2R;
joint2Interval = 1*D2R;
joint3Interval = 2*D2R;
joint1LimitPos = [-60,60]*D2R;
joint2LimitPos = [-30,30]*D2R;
joint3LimitPos = [-60,60]*D2R;
joint1Range = joint1LimitPos(2) - joint1LimitPos(1);
joint2Range = joint2LimitPos(2) - joint2LimitPos(1);
joint3Range = joint3LimitPos(2) - joint3LimitPos(1);
num = int64((joint1Range/joint1Interval+1)*(joint2Range/joint2Interval+1)*(joint3Range/joint3Interval+1));
Pt_Cart = zeros(3,num);
workSpace_ellipsoid = zeros(1,num);
workSpace_cube      = zeros(1,num);
qd = [10 10 10 0 0 0 0]';
vMax = 0.0;
wMax = 0.0;
errPosMax = 0;
m = 1;

%%%%%%%%%%%%%%%%各坐标系初始旋转矩阵%%%%%%%%%%%%%%%
for i = 0:round(joint1Range/joint1Interval)
    jointPos(1) = joint1LimitPos(1) + joint1Interval*i;
    for j = 0:round(joint2Range/joint2Interval)
        jointPos(2) = joint2LimitPos(1) + joint2Interval*j;
        for k = 0:round(joint3Range/joint3Interval)
            jointPos(3) = joint3LimitPos(1) + joint3Interval*k;
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
%{
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

%}
%{
V_Cart_temp = jacobian * qd;
if norm(V_Cart_temp(1:3)) > vMax
    vMax = norm(V_Cart_temp(1:3));
end
if norm(V_Cart_temp(4:6)) > wMax
    wMax = norm(V_Cart_temp(4:6));
end
V_Cart(:,m) = V_Cart_temp;
%}
Pt_Cart_temp = P0t;


%%%%%%%%%%%%%%%%%%基于坐标系0下的笛卡尔位置%%%%%%%%%%%%%%%%%%%
            %Pt_Cart(:,m) = displayRot\P0t;
            m = m + 1;
        end
    end
end
%{
arP = [0.277,0.252,-0.15]';
xmin = min(Pt_Cart(1,:));
xmax = max(Pt_Cart(1,:));
ymin = min(Pt_Cart(2,:));
ymax = max(Pt_Cart(2,:));
zmin = min(Pt_Cart(3,:));
zmax = max(Pt_Cart(3,:));
X0 = (xmin + xmax)/2;
Y0 = (ymin + ymax)/2;
Z0 = (zmin + zmax)/2;
a = (xmax - xmin)/2;
b = (ymax - ymin)/2;
c = (zmax - zmin)/2;
if (arP(1)-X0)^2/a^2 + (arP(2)-Y0)^2/b^2 + (arP(3)-Z0)^2/c^2 <= 1
    inWorkSpace = 1;
else
    inWorkSpace = 0;
end
%%%%%%%%%%%%%%%
for d = 1:(m-1 )
    if (Pt_Cart(1,d)-X0)^2/a^2 + (Pt_Cart(2,d)-Y0)^2/b^2 + (Pt_Cart(3,d)-Z0)^2/c^2 <= 1
        workSpace_ellipsoid(d) = 1;
    else
        workSpace_ellipsoid(d) = 0;
    end
end

for d = 1:(m-1)
    if (Pt_Cart(1,d) <= xmax && Pt_Cart(1,d) >= xmin) && (Pt_Cart(2,d) <= ymax && Pt_Cart(2,d) >= ymin) && (Pt_Cart(3,d) <= zmax && Pt_Cart(3,d) >= zmin)
        workSpace_cube(d) = 1;
    else
        workSpace_cube(d) = 0;
    end
end
%}

