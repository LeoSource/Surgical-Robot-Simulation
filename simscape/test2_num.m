clear
clc
% reference to test2.slx
% 先公式推导得到解析解
% 再根据各关节位置、�?度�?加�?度以及力矩反馈�?辨识得到惯�?参数

% 正常连杆，具有质量和惯量，质心位置与几何中心重合（立方体连杆�?
syms m1 m2 g I1 I2
syms q1 q2 dq1 dq2 ddq1 ddq2 
syms v1 v2 w1 w2
syms x1(t) dx1 ddx1 x2(t) dx2 ddx2

dx1=diff(x1,t); dx2=diff(x2,t);
ddx1=diff(x1,t,t); ddx2=diff(x2,t,t);

len1 = 0.5; len2 = 0.5;
w1 = dx1; w2 = dx1+dx2;
v1 = 0; v2 = v1+w1*len1;
vc1 = v1+w1*len1/2;
vc2_vec = [-sin(x1)*len1*dx1-len2/2*sin(x1+x2)*(dx1+dx2); ...
                  cos(x1)*len1*dx1+len2/2*cos(x1+x2)*(dx1+dx2)];
vc2 = norm(vc2_vec);
%动能
T1 = 1/2*m1*vc1^2+1/2*I1*w1^2;
T2 = 1/2*m2*vc2^2+1/2*I2*w2^2;
T = T1+T2;
%重力势能
U1 = 1/2*m1*g*len1*sin(x1);
U2 = m2*g*(len1*sin(x1)+len2/2*sin(x2+x1));
U = U1 + U2;
%拉格朗日方程
E = T - U;
E = subs(E,{x1,dx1,ddx1,x2,dx2,ddx2},{q1,dq1,ddq1,q2,dq2,ddq2});
dEddq1 = diff(E,dq1);
dEddq1 = subs(dEddq1,{q1,dq1,ddq1,q2,dq2,ddq2},{x1,dx1,ddx1,x2,dx2,ddx2});
dEddq1dt = diff(dEddq1,t);
dEddq1dt = subs(dEddq1dt,{x1,dx1,ddx1,x2,dx2,ddx2},{q1,dq1,ddq1,q2,dq2,ddq2});

dEddq2 = diff(E,dq2);
dEddq2 = subs(dEddq2,{q1,dq1,ddq1,q2,dq2,ddq2},{x1,dx1,ddx1,x2,dx2,ddx2});
dEddq2dt = diff(dEddq2,t);
dEddq2dt = subs(dEddq2dt,{x1,dx1,ddx1,x2,dx2,ddx2},{q1,dq1,ddq1,q2,dq2,ddq2});

dEdq1 = diff(E,q1);
dEdq2 = diff(E,q2);
tau1 = dEddq1dt - dEdq1;
tau2 = dEddq2dt - dEdq2;
%% 化为矩阵形式
%%coeffs(a*x^3+2*x+1,x,'All')，能够提取出为零的系�?
tau1_m1 = diff(tau1,m1);
tau1_m2 = diff(tau1,m2);
tau1_I1 = diff(tau1,I1);
tau1_I2 = diff(tau1,I2);
tau2_m1 = diff(tau2,m1);
tau2_m2 = diff(tau2,m2);
tau2_I1 = diff(tau2,I1);
tau2_I2 = diff(tau2,I2);
Y = [tau1_m1, tau1_m2, tau1_I1, tau1_I2;
        tau2_m1, tau2_m2, tau2_I1, tau2_I2];
Y = simplify(Y);
%% 数�?求解，辨识惯性参�?
%{
g = 9.8 ;len1= 0.5; len2 = 0.5;
Y_num = [];
tau_num = [];
for i=1:length(tout)
    q1 = q1_num(i);
    q2 = q2_num(i);
    dq1 = dq1_num(i);
    dq2 = dq2_num(i);
    ddq1 = ddq1_num(i);
    ddq2 = ddq2_num(i);
    Y11 = (ddq1*len1^2)/4 + (g*cos(q1)*len1)/2;
    Y12 = g*((len2*cos(q1 + q2))/2 + len1*cos(q1)) + sign(dq1*len1*cos(q1) + ...
               (len2*cos(q1 + q2)*(dq1 + dq2))/2)^2*((len2*cos(q1 + q2))/2 + ...
               len1*cos(q1))*(ddq1*len1*cos(q1) - (len2*sin(q1 + q2)*(dq1 + dq2)^2)/2 - ...
               dq1^2*len1*sin(q1) + (len2*cos(q1 + q2)*(ddq1 + ddq2))/2) + ...
               sign(dq1*len1*sin(q1) + (len2*sin(q1 + q2)*(dq1 + dq2))/2)^2*((len2*sin(q1 + q2))/2 + ...
               len1*sin(q1))*(ddq1*len1*sin(q1) + dq1^2*len1*cos(q1) + ...
               (len2*sin(q1 + q2)*(ddq1 + ddq2))/2 + (len2*cos(q1 + q2)*(dq1 + dq2)^2)/2) + 2*abs(dq1*len1*cos(q1) + (len2*cos(q1 + q2)*(dq1 + dq2))/2)*dirac(dq1*len1*cos(q1) + (len2*cos(q1 + q2)*(dq1 + dq2))/2)*((len2*cos(q1 + q2))/2 + len1*cos(q1))*(ddq1*len1*cos(q1) - (len2*sin(q1 + q2)*(dq1 + dq2)^2)/2 - dq1^2*len1*sin(q1) + (len2*cos(q1 + q2)*(ddq1 + ddq2))/2) + 2*abs(dq1*len1*sin(q1) + (len2*sin(q1 + q2)*(dq1 + dq2))/2)*dirac(dq1*len1*sin(q1) + (len2*sin(q1 + q2)*(dq1 + dq2))/2)*((len2*sin(q1 + q2))/2 + len1*sin(q1))*(ddq1*len1*sin(q1) + dq1^2*len1*cos(q1) + (len2*sin(q1 + q2)*(ddq1 + ddq2))/2 + (len2*cos(q1 + q2)*(dq1 + dq2)^2)/2);
    Y13 = ddq1;
    Y14 = ddq1 + ddq2;
    Y21 = 0;
    Y22 = (g*len2*cos(q1 + q2))/2 + (len2*sign(dq1*len1*cos(q1) + (len2*cos(q1 + q2)*(dq1 + dq2))/2)^2*cos(q1 + q2)*(ddq1*len1*cos(q1) - (len2*sin(q1 + q2)*(dq1 + dq2)^2)/2 - dq1^2*len1*sin(q1) + (len2*cos(q1 + q2)*(ddq1 + ddq2))/2))/2 + (len2*sign(dq1*len1*sin(q1) + (len2*sin(q1 + q2)*(dq1 + dq2))/2)^2*sin(q1 + q2)*(ddq1*len1*sin(q1) + dq1^2*len1*cos(q1) + (len2*sin(q1 + q2)*(ddq1 + ddq2))/2 + (len2*cos(q1 + q2)*(dq1 + dq2)^2)/2))/2 + len2*abs(dq1*len1*cos(q1) + (len2*cos(q1 + q2)*(dq1 + dq2))/2)*dirac(dq1*len1*cos(q1) + (len2*cos(q1 + q2)*(dq1 + dq2))/2)*cos(q1 + q2)*(ddq1*len1*cos(q1) - (len2*sin(q1 + q2)*(dq1 + dq2)^2)/2 - dq1^2*len1*sin(q1) + (len2*cos(q1 + q2)*(ddq1 + ddq2))/2) + len2*abs(dq1*len1*sin(q1) + (len2*sin(q1 + q2)*(dq1 + dq2))/2)*dirac(dq1*len1*sin(q1) + (len2*sin(q1 + q2)*(dq1 + dq2))/2)*sin(q1 + q2)*(ddq1*len1*sin(q1) + dq1^2*len1*cos(q1) + (len2*sin(q1 + q2)*(ddq1 + ddq2))/2 + (len2*cos(q1 + q2)*(dq1 + dq2)^2)/2);
    Y23 = 0;
    Y24 = ddq1 + ddq2;
    tmpY = [Y11 Y12 Y13 Y14;
                  Y21 Y22 Y23 Y24];
    Y_num = [Y_num;tmpY];
    tmp_tau = [tau1_num(i);tau2_num(i)];
    tau_num = [tau_num;tmp_tau];    
end
p = inv(Y_num(3:end,:)'*Y_num(3:end,:))*Y_num(3:end,:)'*tau_num(3:end)
%}
%% 根据Y矩阵表达式，利用�?��二乘法求解得到惯性参�?
Y_num = [];
tau_num = [];
g = 9.8; len1 = 0.5; len2 = 0.5;
for i=1:length(tout)
    q1 = q1_num(i);
    q2 = q2_num(i);
    dq1 = dq1_num(i);
    dq2 = dq2_num(i);
    ddq1 = ddq1_num(i);
    ddq2 = ddq2_num(i);
    %eval函数将自变量用数值代替，并且将结果由sym->double
    tmpY = eval(Y);
    Y_num = [Y_num;tmpY];
    
    tmp_tau = [tau1_num(i);tau2_num(i)];
    tau_num = [tau_num;tmp_tau];        
end
%惯�?参数
p = inv(Y_num(3:end,:)'*Y_num(3:end,:))*Y_num(3:end,:)'*tau_num(3:end)
