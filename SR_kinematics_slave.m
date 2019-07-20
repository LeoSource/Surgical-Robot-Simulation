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
                      
Jw       = zeros(3,6);
Jv       = zeros(3,6);


%jointPos =  [0 -50 0*180/pi 0 0 0]/180*pi;
% jointPos =  [-0.0813875347, 0.2458874, 0.237897024, -0.0495979339, -0.28532213, -0.5105175]; %normal running
%jointPos =  [-0.101581164, 0.277775526, 0.235600367, -0.170704067, -1.2165035, -0.6427434]; %abnormal posture
TIME = 0.5;
VMAX = 0.12;
jointPos =  [-0.0, 0, 0, 0, 0, 0]'; 
jointVel = [ 0, 0, 0, 0, 0, 0 ]';
i = 0;
for time = 0:0.001:3
    i = i + 1;
    if time < TIME
        jointVel(3) = VMAX / TIME * time;
    elseif (time <= 2.5) && (time >= 0.5)
        jointVel(3) = VMAX;
    elseif (time > 2.5) && (time <= 3)
        jointVel(3) = -VMAX/TIME * (time - 3);
    end

    jointPos = jointPos + jointVel * 0.001;
%%%%%%%%%%%%%%%%%%%%%%%%%singular configuration%%%%%%%%%%%%%%%%%%%%%%%%%%%
%1. The joint3 is at the maximum position(0.338+0.009), then joint2 and joint5
%   have the same coordinate system;
%2. The joint2 is at the position of -120degree, then the slave loses the
%   velocity of x direction;
%3. The slave configuration is [-0.06845582, 0.277778, 0.233889028, 0.15274711,
%   -1.65579963, -0.442598224], then the end-effctor is at the extension 
%   line of  joint1 direction, that causes the slave loses the velocity of
%   x direction.
%%%%%%%%%%%%%%%%%%%%%%%singular configuration%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%%%%%%%目标坐标系At的速度/角速度由之前的A1~A6所有坐标系的运动叠加得到%%%%%%%%
%%%%%%%wt = w1 + w2 + w3 + w4 + w5 + w6
%%%%%%%vt = w1×P1t + w2×P2t + v3 + w4×P4t + w5×P5t + w6×P6t 
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
cartVel_tmp = jacobian * jointVel;
%%%%abnormal posture
%errCartVel = [ -0.159515023, 2.12388039, 0.419873, 0.4929581, 0.0461047664, 0.004384071 ]';
%tarCartVel = [ -0.00669588242, -0.0135961426, 0.0153179374, 0.2232777, 0.08158365, -0.560607851 ]';
%%%%normal running
% errCartVel = [ 0.000106869265, -0.000214576721, -0.00005364418, 0, 0, 0 ]';
% tarCartVel = [ 0.00336327846, 0.0208354127, 0.009880804, 0.0505033024, 0.0528136268,0.173714921 ]';
%%%%SVD validate
errCartVel = [ -0.0383935869, 1.74638629, 0.354731083, 0.7148393, 0.125727221, -0.104529321 ]';
tarCartVel = [ -0.00451020524, -0.0131495409, 0.0240823478, 0.221941456, 0.0656575,-0.5328697 ]';

refCartVel = errCartVel + tarCartVel;
baseJointVel = pinv(jacobian) * refCartVel;
[U,S,V] = svd(jacobian);


    cartVel(:,i) = cartVel_tmp;
    cartPos(:,i) = P0t;
end







% V        = [0 v 0 0 0 0]';  
% Vt       = jacobian * V;
% vt       = Vt(1:3);
% wt       = Vt(4:6);

% output_v(:,i) = vt;
% end
%%%%%%%%%%%机械臂运动显示%%%%%%%%%%%
% plot3([0,P01(1),P02(1),P03(1)],[0,P01(2),P02(2),P03(2)],[0,P01(3),P02(3),P03(3)],'LineWidth',3);
% grid on;
% xlabel('x(m)');  ylabel('y(m)');  zlabel('z(m)');
