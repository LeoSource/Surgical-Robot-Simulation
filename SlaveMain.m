clear
clc

%%%%%%%%%%%%%变量定义%%%%%%%%%%
global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R6t_orl
global P01_orl P12_orl P23_orl P34_orl P45_orl P56_orl P6t_orl
global P01 P02 P03 P04 P05 P06 P0t
global R01 R02 R03 R04 R05 R06
global Jw Jv


slave_Length = [0.16 0.7369 0 0.338 0 0.009 0];   %%各坐标系原点初始间距：坐标系0原点到坐标系1原点向量在坐标系0的表示，后续以此类推
% jointPos = [0 0 0*180/pi 0 90 0]*pi/180;
% nothing = SlaveInit(slave_Length);
% CartOri = SlaveOrientation(jointPos);
% CartPos = SlavePosition(jointPos);

%{
%% 1. 笛卡尔全范围工作空间求解
jointPos = [0 0 0 0 0 0]*pi/180;
nothing = SlaveInit(slave_Length);

workSpace_ellipsoid = zeros(1,num);
workSpace_cube      = zeros(1,num);
m = 1;

for i = 0:(jointRange(1)/jointInterval(1))
    jointPos(1) = joint1LimitPos(1) + jointInterval(1)*i;
    for j = 0:round(jointRange(2)/jointInterval(2))
        jointPos(2) = joint2LimitPos(1) + jointInterval(2)*j;
        for k = 0:jointRange(3)/jointInterval(3)
            jointPos(3) = joint3LimitPos(1) + jointInterval(3)*k;
            
            CartOri = SlaveOrientation(jointPos);
            CartPos = SlavePosition(jointPos);
            Pt_Cart(:,m) = CartPos;
            m = m + 1;
        end
    end
end

jacobian = SlaveDiffKinematics(0);
ar_jointPos = [0, 0, 0, 0, 0, 0];
CartOri_temp = SlaveOrientation(ar_jointPos);
CartPos_temp = SlavePosition(ar_jointPos);
arP = CartPos_temp;
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
if ( (arP(1)-X0)^2/a^2 + (arP(2)-Y0)^2/b^2 + (arP(3)-Z0)^2/c^2 <= 1 )
    inWorkSpace = 1;
else
    inWorkSpace = 0;
end
%%%%%%%%椭球空间过小，无法包围正常工作空间%%%%%%%%%
for d = 1:(m-1)
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

%% 2. 笛卡尔空间非预期运动安全检测阈值求解
%{
jointPos = [0 0 0 0 0 0]*pi/180;
nothing = SlaveInit(slave_Length);

jointInterval = [0.04, 0.04, 0.002];
joint1LimitPos = [-2.08,2.08];
joint2LimitPos = [-1.88,0.36];
joint3LimitPos = [0,0.338];
jointRange = zeros(1,3);
jointRange(1) = joint1LimitPos(2) - joint1LimitPos(1);
jointRange(2) = joint2LimitPos(2) - joint2LimitPos(1);
jointRange(3) = joint3LimitPos(2) - joint3LimitPos(1);
num = int64((jointRange(1)/jointInterval(1)+1)*(round(jointRange(2)/jointInterval(2))+1)*(jointRange(3)/jointInterval(3)+1));
V_Cart = zeros(6,num);
qd = [0.3,0.3,0.05,0,0,0]';

CartOri = zeros(3,3);
V_Cart_temp = zeros(6,1);
vMax = 0.0;
wMax = 0.0;
m = 1;
for i = 0:(jointRange(1)/jointInterval(1))
    jointPos(1) = joint1LimitPos(1) + jointInterval(1)*i;
    for j = 0:round(jointRange(2)/jointInterval(2))
        jointPos(2) = joint2LimitPos(1) + jointInterval(2)*j;
        for k = 0:jointRange(3)/jointInterval(3)
            jointPos(3) = joint3LimitPos(1) + jointInterval(3)*k;
            
            CartOri = SlaveOrientation(jointPos);
            CartPos = SlavePosition(jointPos);
            jacobian = SlaveDiffKinematics(0);
            
            V_Cart_temp = jacobian *qd;
            if ( norm(V_Cart_temp(1:3)) > vMax )
                vMax = norm(V_Cart_temp(1:3));
            end
            if ( norm(V_Cart_temp(4:6)) > wMax )
                wMax = norm(V_Cart_temp(4:6));
            end

            V_Cart(:,m) =V_Cart_temp;
            m = m + 1;
        end
    end
end
%}
%% 3. 笛卡尔空间位置偏差阈值求解
%{
jointPos = [0 0 0 0 0 0]*pi/180;
cmdJointPosS = [0 0 0 0 0 0]*pi/180;
cmdJointPosB = [0 0 0 0 0 0]*pi/180;
threshold = [0.2 0.2 0.02 0 0 0];
nothing = SlaveInit(slave_Length);

jointInterval = [0.04, 0.04, 0.002];
joint1LimitPos = [-2.08,2.08];
joint2LimitPos = [-1.88,0.36];
joint3LimitPos = [0,0.338];
jointRange = zeros(1,3);
jointRange(1) = joint1LimitPos(2) - joint1LimitPos(1);
jointRange(2) = joint2LimitPos(2) - joint2LimitPos(1);
jointRange(3) = joint3LimitPos(2) - joint3LimitPos(1);
num = int64((jointRange(1)/jointInterval(1)+1)*(round(jointRange(2)/jointInterval(2))+1)*(jointRange(3)/jointInterval(3)+1));
Pt_Cart = zeros(3,num);

CartOri = zeros(3,3);
Pt_Cart_temp = zeros(3,1);
errPosMax = 0;
m = 1;
for i = 0:(jointRange(1)/jointInterval(1))
    jointPos(1) = joint1LimitPos(1) + jointInterval(1)*i;
    for j = 0:round(jointRange(2)/jointInterval(2))
        jointPos(2) = joint2LimitPos(1) + jointInterval(2)*j;
        for k = 0:jointRange(3)/jointInterval(3)
            jointPos(3) = joint3LimitPos(1) + jointInterval(3)*k;
            
            CartOri = SlaveOrientation(jointPos);
            CartPos = SlavePosition(jointPos);
            Pt_Cart_temp = CartPos;
            
            cmdJointPosS(1) = jointPos(1) - threshold(1);
            cmdJointPosS(2) = jointPos(2) - threshold(2);
            cmdJointPosS(3) = jointPos(3) - threshold(3);
            cmdCartOriS = SlaveOrientation(cmdJointPosS);
            cmdCartPosS = SlavePosition(cmdJointPosS);
            
            cmdJointPosB(1) = jointPos(1) + threshold(1);
            cmdJointPosB(2) = jointPos(2) + threshold(2);
            cmdJointPosB(3) = jointPos(3) + threshold(3);
            cmdCartOriB = SlaveOrientation(cmdJointPosB);
            cmdCartPosB = SlavePosition(cmdJointPosB);
            
            if norm(cmdCartPosS - Pt_Cart_temp) > errPosMax
                errPosMax = norm(cmdCartPosS - Pt_Cart_temp);
            end
            if norm(cmdCartPosB - Pt_Cart_temp) > errPosMax
                errPosMax = norm(cmdCartPosB - Pt_Cart_temp);
            end
            
            Pt_Cart(:,m) =Pt_Cart_temp;
            m = m + 1;
        end
    end
end
%}

%% 4. 笛卡尔速度超限阈值求解
%{
jointPos = [0 0 0 0 0 0]*pi/180;
nothing = SlaveInit(slave_Length);

jointInterval = [0.04, 0.04, 0.002];
joint1LimitPos = [-2.08,2.08];
joint2LimitPos = [-1.88,0.36];
joint3LimitPos = [0,0.338];
jointRange = zeros(1,3);
jointRange(1) = joint1LimitPos(2) - joint1LimitPos(1);
jointRange(2) = joint2LimitPos(2) - joint2LimitPos(1);
jointRange(3) = joint3LimitPos(2) - joint3LimitPos(1);
num = int64((jointRange(1)/jointInterval(1)+1)*(round(jointRange(2)/jointInterval(2))+1)*(jointRange(3)/jointInterval(3)+1));
V_Cart = zeros(6,num);
qd = [2.2,2.2,0.33,0,0,0]';%%先设定关节端最大柔顺速度

CartOri = zeros(3,3);
V_Cart_temp = zeros(6,1);
vMax = 0.0;
wMax = 0.0;
m = 1;
for i = 0:(jointRange(1)/jointInterval(1))
    jointPos(1) = joint1LimitPos(1) + jointInterval(1)*i;
    for j = 0:round(jointRange(2)/jointInterval(2))
        jointPos(2) = joint2LimitPos(1) + jointInterval(2)*j;
        for k = 0:jointRange(3)/jointInterval(3)
            jointPos(3) = joint3LimitPos(1) + jointInterval(3)*k;
            
            CartOri = SlaveOrientation(jointPos);
            CartPos = SlavePosition(jointPos);
            jacobian = SlaveDiffKinematics(0);
            
            V_Cart_temp = jacobian *qd;
            if ( norm(V_Cart_temp(1:3)) > vMax )
                vMax = norm(V_Cart_temp(1:3));
            end
            if ( norm(V_Cart_temp(4:6)) > wMax )
                wMax = norm(V_Cart_temp(4:6));
            end

            V_Cart(:,m) =V_Cart_temp;
            m = m + 1;
        end
    end
end
%}
%% 笛卡尔空间轨迹规划

jointPos = [0 0 0*180/pi 0 0 0]'*pi/180;
nothing = SlaveInit(slave_Length);
R0 = SlaveOrientation(jointPos);
X0 = SlavePosition(jointPos);

Rd = rotx(0);
Xd = X0 + [0.01, 0, 0.4]';

Rk_theta = Rd * R0';
thetaf =  acos((Rk_theta(1,1) + Rk_theta(2,2) + Rk_theta(3,3) - 1 ) / 2);
theta0 = 0;
K_tmp = [Rk_theta(3,2)-Rk_theta(2,3);Rk_theta(1,3)-Rk_theta(3,1);Rk_theta(2,1)-Rk_theta(1,2)];
if thetaf == 0
    K = [0 0 0]';
else
    K = K_tmp / (2*sin(thetaf));
end

%%%%%三次多项式插值%%%%
TIME = 2;
interPara_CartX.a0 = X0(1);
interPara_CartX.a1 = 0;
interPara_CartX.a2 = 3/TIME^2 * (Xd(1) - X0(1));
interPara_CartX.a3 = -2/TIME^3 * (Xd(1) - X0(1));
interPara_CartY.a0 = X0(2);
interPara_CartY.a1 = 0;
interPara_CartY.a2 = 3/TIME^2 * (Xd(2) - X0(2));
interPara_CartY.a3 = -2/TIME^3 * (Xd(2) - X0(2));
interPara_CartZ.a0 = X0(3);
interPara_CartZ.a1 = 0;
interPara_CartZ.a2 = 3/TIME^2 * (Xd(3) - X0(3));
interPara_CartZ.a3 = -2/TIME^3 * (Xd(3) - X0(3));
interPara_Rot.a0 = theta0;
interPara_Rot.a1 = 0;
interPara_Rot.a2 = 3/TIME^2 * (thetaf - theta0);
interPara_Rot.a3 = -2/TIME^3 * (thetaf - theta0);

for i=1:1000
    
    t = i * 2/1000;
    theta = interPara_Rot.a0 + interPara_Rot.a1 * t + interPara_Rot.a2 * t^2 + interPara_Rot.a3 * t^3;
    theta_ = interPara_Rot.a1 + 2 * interPara_Rot.a2 * t + 3 * interPara_Rot.a3 * t^2;
    vx = interPara_CartX.a1 + 2 * interPara_CartX.a2 * t + 3 * interPara_CartX.a3 * t^2;
    vy = interPara_CartY.a1 + 2 * interPara_CartY.a2 * t + 3 * interPara_CartY.a3 * t^2;
    vz = interPara_CartZ.a1 + 2 * interPara_CartZ.a2 * t + 3 * interPara_CartZ.a3 * t^2;
    
    SlaveOrientation(jointPos);
    SlavePosition(jointPos);
    jacobian = SlaveDiffKinematics(0);
    V = [ vx vy vz]';
    W = theta_ * K;
    jointVel = pinv(jacobian) * [V;W];
    jointPos = jointVel * 2/1000 + jointPos;
    
    jointVel_tmp(:,i) = jointVel;
    jointPos_tmp(:,i) = jointPos;
    
end
posErr = Xd - SlavePosition(jointPos)
rotErr = Rd - SlaveOrientation(jointPos)
time = 2/1000:2/1000:2;
figure(1)
plot(time,jointVel_tmp(1,:),time,jointVel_tmp(2,:),time,jointVel_tmp(3,:));
legend('joint1Vel','joint2Vel','joint3Vel');

figure(2)
plot(time,jointVel_tmp(4,:),time,jointVel_tmp(5,:),time,jointVel_tmp(6,:));
legend('joint4Vel','joint5Vel','joint6Vel');

figure(3)
plot(time,jointPos_tmp(1,:),time,jointPos_tmp(2,:),time,jointPos_tmp(3,:));
legend('joint1Pos','joint2Pos','joint3Pos');

figure(4)
plot(time,jointPos_tmp(4,:),time,jointPos_tmp(5,:),time,jointPos_tmp(6,:));
legend('joint4Pos','joint5Pos','joint6Pos');
