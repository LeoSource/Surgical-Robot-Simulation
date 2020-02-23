function [ cond_Yr ] = traj_ext( ~ )
%   UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明

    % 两连杆机械臂激励轨迹优化
    % reference to test2.slx and test2_num.m

    q = sym(zeros(1,2));
    dq = sym(zeros(1,2));
    ddq = sym(zeros(1,2));
    syms q1_0 q2_0
    syms t
    syms g
    a = sym('a',[2,5]);
    b = sym('b',[2,5]);
    q0 = [q1_0 q2_0];
    fourier_num = 5;
    jnt_num = 2;

    len1 = 0.5; len2 = 0.5; %杆长
    Ts = 0.01;%采样时间
    Tf = 10;%总运行时间
    wf = 0.1;%基准频率

    %傅里叶参数初始化
    for i=1:jnt_num
        for j=1:fourier_num
            q(i) = a(i,j)/(wf*j)*sin(wf*j*t) - b(i,j)/(wf*j)*cos(wf*j*t) +q(i);
            dq(i) = a(i,j)*cos(wf*j*t) + b(i,j)*sin(wf*j*t) + dq(i);
            ddq(i) = -a(i,j)*wf*j*sin(wf*j*t) + b(i,j)*wf*j*cos(wf*j*t) + ddq(i);
        end
        q(i) = q(i) + q0(i);
    end

    Y11 = (ddq(1)*len1^2)/4 + (g*cos(q(1))*len1)/2;
    Y12 = g*((len2*cos(q(1) + q(2)))/2 + len1*cos(q(1))) + sign(dq(1)*len1*cos(q(1)) + ...
               (len2*cos(q(1) + q(2))*(dq(1) + dq(2)))/2)^2*((len2*cos(q(1) + q(2)))/2 + ...
               len1*cos(q(1)))*(ddq(1)*len1*cos(q(1)) - (len2*sin(q(1) + q(2))*(dq(1) + dq(2))^2)/2 - ...
               dq(1)^2*len1*sin(q(1)) + (len2*cos(q(1) + q(2))*(ddq(1) + ddq(2)))/2) + ...
               sign(dq(1)*len1*sin(q(1)) + (len2*sin(q(1) + q(2))*(dq(1) + dq(2)))/2)^2*((len2*sin(q(1) + q(2)))/2 + ...
               len1*sin(q(1)))*(ddq(1)*len1*sin(q(1)) + dq(1)^2*len1*cos(q(1)) + ...
               (len2*sin(q(1) + q(2))*(ddq(1) + ddq(2)))/2 + (len2*cos(q(1) + q(2))*(dq(1) + dq(2))^2)/2) + 2*abs(dq(1)*len1*cos(q(1)) + (len2*cos(q(1) + q(2))*(dq(1) + dq(2)))/2)*dirac(dq(1)*len1*cos(q(1)) + (len2*cos(q(1) + q(2))*(dq(1) + dq(2)))/2)*((len2*cos(q(1) + q(2)))/2 + len1*cos(q(1)))*(ddq(1)*len1*cos(q(1)) - (len2*sin(q(1) + q(2))*(dq(1) + dq(2))^2)/2 - dq(1)^2*len1*sin(q(1)) + (len2*cos(q(1) + q(2))*(ddq(1) + ddq(2)))/2) + 2*abs(dq(1)*len1*sin(q(1)) + (len2*sin(q(1) + q(2))*(dq(1) + dq(2)))/2)*dirac(dq(1)*len1*sin(q(1)) + (len2*sin(q(1) + q(2))*(dq(1) + dq(2)))/2)*((len2*sin(q(1) + q(2)))/2 + len1*sin(q(1)))*(ddq(1)*len1*sin(q(1)) + dq(1)^2*len1*cos(q(1)) + (len2*sin(q(1) + q(2))*(ddq(1) + ddq(2)))/2 + (len2*cos(q(1) + q(2))*(dq(1) + dq(2))^2)/2);
    Y13 = ddq(1);
    Y14 = ddq(1) + ddq(2);
    Y21 = 0;
    Y22 = (g*len2*cos(q(1) + q(2)))/2 + (len2*sign(dq(1)*len1*cos(q(1)) + (len2*cos(q(1) + q(2))*(dq(1) + dq(2)))/2)^2*cos(q(1) + q(2))*(ddq(1)*len1*cos(q(1)) - (len2*sin(q(1) + q(2))*(dq(1) + dq(2))^2)/2 - dq(1)^2*len1*sin(q(1)) + (len2*cos(q(1) + q(2))*(ddq(1) + ddq(2)))/2))/2 + (len2*sign(dq(1)*len1*sin(q(1)) + (len2*sin(q(1) + q(2))*(dq(1) + dq(2)))/2)^2*sin(q(1) + q(2))*(ddq(1)*len1*sin(q(1)) + dq(1)^2*len1*cos(q(1)) + (len2*sin(q(1) + q(2))*(ddq(1) + ddq(2)))/2 + (len2*cos(q(1) + q(2))*(dq(1) + dq(2))^2)/2))/2 + len2*abs(dq(1)*len1*cos(q(1)) + (len2*cos(q(1) + q(2))*(dq(1) + dq(2)))/2)*dirac(dq(1)*len1*cos(q(1)) + (len2*cos(q(1) + q(2))*(dq(1) + dq(2)))/2)*cos(q(1) + q(2))*(ddq(1)*len1*cos(q(1)) - (len2*sin(q(1) + q(2))*(dq(1) + dq(2))^2)/2 - dq(1)^2*len1*sin(q(1)) + (len2*cos(q(1) + q(2))*(ddq(1) + ddq(2)))/2) + len2*abs(dq(1)*len1*sin(q(1)) + (len2*sin(q(1) + q(2))*(dq(1) + dq(2)))/2)*dirac(dq(1)*len1*sin(q(1)) + (len2*sin(q(1) + q(2))*(dq(1) + dq(2)))/2)*sin(q(1) + q(2))*(ddq(1)*len1*sin(q(1)) + dq(1)^2*len1*cos(q(1)) + (len2*sin(q(1) + q(2))*(ddq(1) + ddq(2)))/2 + (len2*cos(q(1) + q(2))*(dq(1) + dq(2))^2)/2);
    Y23 = 0;
    Y24 = ddq(1) + ddq(2);
    Y = [Y11 Y12 Y13 Y14; Y21 Y22 Y23 Y24];
    Yr = subs(Y,t,0);

    for i=1:(Tf/Ts)
        tmpY = subs(Y,t,i*Ts);
        Yr = [Yr;tmpY];
    end
    
    Yr_cond = cond(Yr);

end

