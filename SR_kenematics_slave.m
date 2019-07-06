%%%%%%%%%%%%%%�����˶�ѧģ��%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
%%%%%%%%%%%%%%������ʼ��%%%%%%%%%%%%%
theta1   =  30/180*pi;
length   =  [0.16 0.7369 0 0.338 0 0.009 0];   %%������ϵԭ���ʼ��ࣺ����ϵ0ԭ�㵽����ϵ1ԭ������������ϵ0�ı�ʾ�������Դ�����

RotZ     =  @(theta)([cos(theta) -sin(theta) 0
                      sin(theta)  cos(theta) 0
                          0           0      1]);
                      
Jw       = zeros(3,6);
Jv       = zeros(3,6);

sampleTime = 0.01;
jointPos =  [0 0 0*180/pi 0 0 0]/180*pi;
output_v = zeros(3,1000);
output_p = zeros(3,1000);
output_w = zeros(3,1000);
%%%%%%%%%%%%%%%%%%%������ϵ��ʼ��ת����%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%����ϵԭ��λ��%%%%%%%%%%%%%%%%%%%%%%
P01_orl  =  [0 0 -length(1)]';  %%����ϵ1��ԭ��������ϵ0�еı�ʾ
P12_orl  =  [0 0 length(2)]';
P23_orl  =  [0 0 0]';
P34_orl  =  [0 0 -length(4)]';
P45_orl  =  [0 0 0]';
P56_orl  =  [0 -length(6) 0]';
P6t_orl  =  [0 0 0]';

%%%%%%%%%%%%%%������ϵ��Ի�����ϵ����ת����%%%%%%%%%%%%%%%%
v      = 2/180*pi;
 for i = 1:10/sampleTime
     jointPos(2) = jointPos(2) + v*0.01;
R01     = R01_orl;                         %%����ϵ1�������ϵ0����ת����
R02     = R01*RotZ(jointPos(1))*R12_orl;   %%����ϵ2�������ϵ0����ת����
R03     = R02*RotZ(jointPos(2))*R23_orl;
R04     = R03*RotZ(0)*R34_orl;             %%�ؽ�3λ�ƶ��ؽڣ�δ��������ϵ��ת
R05     = R04*RotZ(jointPos(4))*R45_orl;
R06     = R05*RotZ(jointPos(5))*R56_orl;
R0t     = R06*RotZ(jointPos(6))*R6t_orl;

%%%%%%%%%%%%%������ϵԭ����Ի�����ϵԭ��ʸ���ڻ�����ϵ�±�ʾ%%%%%%%%%%%%%%%
P01    = P01_orl;
P02    = P01 + R01*RotZ(jointPos(1))*P12_orl;
P03    = P02 + R02*RotZ(jointPos(2))*P23_orl;
P04    = P03 + R03*([0 ; 0; jointPos(3)] + P34_orl);
P05    = P04 + R04*RotZ(jointPos(4))*P45_orl;
P06    = P05 + R05*RotZ(jointPos(5))*P56_orl;
P0t    = P06 + R06*RotZ(jointPos(6))*P6t_orl;

output_p(:,i) = P0t;
output_p3(:,i) = P03;
output_p4(:,i) = P04;
%%%%%%%%%%%%%%%%%�˶�ѧ���ſ˱Ⱦ������%%%%%%%%%%%%%%%%%
%%%%%%%Ŀ������ϵAt���ٶ�/���ٶ���֮ǰ��A1~A6��������ϵ���˶����ӵõ�%%%%%%%%
%%%%%%%wt = w1 + w2 + w3 + w4 + w5 + w6
%%%%%%%vt = w1��P17 + w2��P27 + v3 + w4��P47 + w5��P57 + w6��P67 
Ez     = [0 0 1]';
Jw(:,1)  = R01*Ez;
Jw(:,2)  = R02*Ez;
Jw(:,3)  = zeros(3,1);%�ؽ�3Ϊ�ƶ��ؽ�
Jw(:,4)  = R04*Ez;
Jw(:,5)  = R05*Ez;
Jw(:,6)  = R06*Ez;

Jv(:,1)  = cross(Jw(:,1),(P0t - P01));
Jv(:,2)  = cross(Jw(:,2),(P0t - P02));
Jv(:,3)  = R03*Ez;%�ؽ�3Ϊ�ƶ��ؽ�
Jv(:,4)  = cross(Jw(:,4),(P0t - P04));
Jv(:,5)  = cross(Jw(:,5),(P0t - P05));
Jv(:,6)  = cross(Jw(:,6),(P0t - P06));

jacobian = [Jv ; Jw];

V        = [0 v 0 0 0 0]';  
Vt       = jacobian * V;
vt       = Vt(1:3);
wt       = Vt(4:6);

output_v(:,i) = vt;
end
%%%%%%%%%%%��е���˶���ʾ%%%%%%%%%%%
% plot3([0,P01(1),P02(1),P03(1)],[0,P01(2),P02(2),P03(2)],[0,P01(3),P02(3),P03(3)],'LineWidth',3);
% grid on;
% xlabel('x(m)');  ylabel('y(m)');  zlabel('z(m)');

%C:\Users\Zhixi\Documents\GitHub\Surgical-Robot-Simulation