clc
clear

%%%%%%%%%%%%%变量定义%%%%%%%%%%
global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R67_orl R78_orl R89_orl R9t_orl display_orl
global P01_orl P12_orl P23_orl P34_orl P45_orl 
global P01 P02 P03 P04 P0t
global R01 R02 R03 R04 R05 R06 R07 R08 R09 R0t displayRot
global Jw Jv

g_length = [ 0.086, 0.0975, (0.28-0.00264), 0.107, (0.26-0.0079), 0.1396
             0.086, 0.0975, (0.28+0.00186), 0.107, (-0.26+0.0048), 0.1396 ];
         

jointPos      =  [ 0 0 0 0 0 0 0]*pi/180;
displayAngle  =  0*pi/180;

%%%%%%%%%%%%%%%%%%%%笛卡尔工作空间求解%%%%%%%%%%%%%%%%%%
%{
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
left_null  = MasterInit(g_length(1,:));
%right_null = MasterInit(g_length(2,:));
for i = 0:round(joint1Range/joint1Interval)
    jointPos(1) = joint1LimitPos(1) + joint1Interval*i;
    for j = 0:round(joint2Range/joint2Interval)
        jointPos(2) = joint2LimitPos(1) + joint2Interval*j;
        for k = 0:round(joint3Range/joint3Interval)
            jointPos(3) = joint3LimitPos(1) + joint3Interval*k;
            
            [leftCartRot,leftCartPos] = MasterKinematics(jointPos,displayAngle); 
            Pt_Cart(:,m) = leftCartPos;
            m = m + 1;
            %rightJacobian = MasterDiffKinematics(0); 
        end
    end
end
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


% nothing = MasterInit(g_length(1,:));
% [leftCartRot,leftCartPos] = MasterKinematics(jointPos,displayAngle); 
% leftJacobian = MasterDiffKinematics(0);

% nothing = MasterInit(g_length(2,:));
% [rightCartRot,rightCartPos] = MasterKinematics(jointPos,displayAngle);
% rightJacobian = MasterDiffKinematics(0);
%}
%% 3. 笛卡尔速度超限阈值
%{
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
qd = [10 10 10 0 0 0 0]';
vMax = 0.0;
wMax = 0.0;
m = 1;

left_null  = MasterInit(g_length(1,:));
for i = 0:round(joint1Range/joint1Interval)
    jointPos(1) = joint1LimitPos(1) + joint1Interval*i;
    for j = 0:round(joint2Range/joint2Interval)
        jointPos(2) = joint2LimitPos(1) + joint2Interval*j;
        for k = 0:round(joint3Range/joint3Interval)
            jointPos(3) = joint3LimitPos(1) + joint3Interval*k;
            
            [leftCartRot,leftCartPos] = MasterKinematics(jointPos,displayAngle); 
            jacobian = MasterDiffKinematics(0);
            
            V_Cart = jacobian * qd;
            if ( norm(V_Cart(1:3)) > vMax )
                vMax = norm(V_Cart(1:3));
            end
            if ( norm(V_Cart(4:6)) > wMax )
                wMax = norm(V_Cart(4:6));
            end
            
            m = m + 1;
        end
    end
end
%}

%% 4. 笛卡尔期望与实际位置偏差超限阈值
%{
cmdJointPosS = [ 0 0 0 0 0 0 0 ]*pi/180;
cmdJointPosB = [ 0 0 0 0 0 0 0 ]*pi/180;
threshold = [0.2 0.2 0.2 0 0 0 0];
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
Pt_Cart_temp = zeros(3,1);
errPosMax = 0;
m = 1;

left_null  = MasterInit(g_length(1,:));
for i = 0:round(joint1Range/joint1Interval)
    jointPos(1) = joint1LimitPos(1) + joint1Interval*i;
    for j = 0:round(joint2Range/joint2Interval)
        jointPos(2) = joint2LimitPos(1) + joint2Interval*j;
        for k = 0:round(joint3Range/joint3Interval)
            jointPos(3) = joint3LimitPos(1) + joint3Interval*k;
            
            [leftCartRot,leftCartPos] = MasterKinematics(jointPos,displayAngle); 
            Pt_Cart_temp = leftCartPos;
            
            cmdJointPosS(1) = jointPos(1) - threshold(1);
            cmdJointPosS(2) = jointPos(2) - threshold(2);
            cmdJointPosS(3) = jointPos(3) - threshold(3);
            [cmdCartRotS,cmdCartPosS] = MasterKinematics(cmdJointPosS,displayAngle);
            
            cmdJointPosB(1) = jointPos(1) + threshold(1);
            cmdJointPosB(2) = jointPos(2) + threshold(2);
            cmdJointPosB(3) = jointPos(3) + threshold(3);
            [cmdCartRotB,cmdCartPosB] = MasterKinematics(cmdJointPosB,displayAngle);
            
            if norm(cmdCartPosS - Pt_Cart_temp) > errPosMax
                errPosMax = norm(cmdCartPosS - Pt_Cart_temp);
            end
            if norm(cmdCartPosB - Pt_Cart_temp) > errPosMax
                errPosMax = norm(cmdCartPosB - Pt_Cart_temp);
            end
            
            Pt_Cart(:,m) = Pt_Cart_temp;
            m = m + 1;
        end
    end
end
%}

%% 5. 笛卡尔期望与实际姿态偏差超限阈值

cmdJointPosS = [ 0 0 0 0 0 0 0 ]*pi/180;
cmdJointPosB = [ 0 0 0 0 0 0 0 ]*pi/180;
threshold = [0.2 0.2 0.2 0.4 0.4 0.4 0.4];
D2R = pi/180;
R2D = 180/pi;
jointInterval = [6*D2R, 6*D2R, 12*D2R, 30*D2R, 27*D2R, 15*D2R, 90*D2R];
joint1LimitPos = [-60,60]*D2R;
joint2LimitPos = [-30,30]*D2R;
joint3LimitPos = [-60,60]*D2R;
joint4LimitPos = [-180,120]*D2R;
joint5LimitPos = [-180,90]*D2R;
joint6LimitPos = [-90,45]*D2R;
joint7LimitPos = [-450,450]*D2R;
jointRange = [( joint1LimitPos(2) - joint1LimitPos(1) ),( joint2LimitPos(2) - joint2LimitPos(1) ),( joint3LimitPos(2) - joint3LimitPos(1) )...
              ( joint4LimitPos(2) - joint4LimitPos(1) ),( joint5LimitPos(2) - joint5LimitPos(1) ),( joint6LimitPos(2) - joint6LimitPos(1) )...
              ( joint7LimitPos(2) - joint7LimitPos(1) )];
num = int64( (jointRange(1)/jointInterval(1)+1)*(jointRange(2)/jointInterval(2)+1)*(jointRange(3)/jointInterval(3)+1)*...
             (jointRange(4)/jointInterval(4)+1)*(jointRange(5)/jointInterval(5)+1)*(jointRange(6)/jointInterval(6)+1)*...
             (jointRange(7)/jointInterval(7)+1) );
%%Rot_Cart = zeros(3,num);
Rot_Cart_temp = zeros(3,1);
errRotMax = 0;
m = 1;

left_null  = MasterInit(g_length(1,:));
for i1 = 0:round(jointRange(1)/jointInterval(1))
    jointPos(1) = joint1LimitPos(1) + jointInterval(1)*i1;
    for i2 = 0:round(jointRange(2)/jointInterval(2))
        jointPos(2) = joint2LimitPos(1) + jointInterval(2)*i2;
        for i3 = 0:round(jointRange(3)/jointInterval(3))
            jointPos(3) = joint3LimitPos(1) + jointInterval(3)*i3;
            for i4 = 0:round(jointRange(4)/jointInterval(4))
                jointPos(4) = joint4LimitPos(1) + jointInterval(4)*i4;
                for i5 = 0:round(jointRange(5)/jointInterval(5))
                    jointPos(5) = joint5LimitPos(1) + jointInterval(5)*i5;
                    for i6 = 0:round(jointRange(6)/jointInterval(6))
                        jointPos(6) = joint6LimitPos(1) + jointInterval(6)*i6;
                        for i7 = 0:round(jointRange(7)/jointInterval(7))
                            jointPos(7) = joint6LimitPos(1) + jointInterval(7)*i7;
                            
                            [leftCartRot,leftCartPos] = MasterKinematics(jointPos,displayAngle); 
                            
                            cmdJointPosS = jointPos - threshold;
                            [cmdCartRotS,cmdCartPosS] = MasterKinematics(cmdJointPosS,displayAngle);
            
                            cmdJointPosB = jointPos + threshold;
                            [cmdCartRotB,cmdCartPosB] = MasterKinematics(cmdJointPosB,displayAngle);
                            
                            errRotS = 0.5*( cross(leftCartRot(:,1),cmdCartRotS(:,1)) + ...
                                            cross(leftCartRot(:,1),cmdCartRotS(:,1)) + ...
                                            cross(leftCartRot(:,1),cmdCartRotS(:,1)) );                                      
                            errRotB = 0.5*( cross(leftCartRot(:,1),cmdCartRotB(:,1)) + ...
                                            cross(leftCartRot(:,1),cmdCartRotB(:,1)) + ...
                                            cross(leftCartRot(:,1),cmdCartRotB(:,1)) );
                                        
                            if norm(errRotS) > errRotMax
                                errRotMax = norm(errRotS);
                            end
                            if norm(errRotB) > errRotMax
                                errRotMax = norm(errRotB);
                            end          
                            m = m +1;
                        end
                    end
                end
            end
        end
    end
end


