clear
clc
% 两连杆机械臂激励轨迹优化
% reference to test2.slx and test2_num.m

q = sym(zeros(1,2));
dq = sym(zeros(1,2));
ddq = sym(zeros(1,2));
syms t
syms g
a = sym('a',[2,5]);
b = sym('b',[2,5]);
q0 = sym('q0',[1,2]);
fourier_num = 5;
jnt_num = 2;

len1 = 0.5; len2 = 0.5; %杆长
g = 9.8;
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

Y11 = ddq(1)+cos(q(1));
Y12 = 0.5*(q(1)+q(2)) + 0.5*(dq(1)+dq(2)) + 0.5*(ddq(1)+ddq(2));
Y13 = ddq(1);
Y14 = ddq(1) + ddq(2);
Y21 = 0;
Y22 = 0.5*(q(1)-q(2)) + 0.5*(dq(1)-dq(2)) + 0.5*(ddq(1)-ddq(2));
Y23 = 0;
Y24 = ddq(1) + ddq(2);
Y = [Y11 Y12 Y13 Y14; Y21 Y22 Y23 Y24];
Yr = subs(Y,t,0);

for i=1:(Tf/Ts)
    tmpY = subs(Y,t,i*Ts);
    Yr = [Yr;tmpY];
end
% Yr_cond = cond(Yr);

%% 傅里叶参数求解
% 限制条件
% q1>= -1.2rad && q1 <= 1.2rad
% q2>= -1.5rad && q2 <= 1.5rad
% dq1>= -1.0rad/s && dq1 <= 1.0rad/s
% dq2>= -1.1rad/s && dq2 <= 1.1rad/s
qmin = [-1.2 -1.5]; qmax = [1.2 1.5];
dqmin = [-1.0 -1.1]; dqmax = [1.0 1.1];
X = [reshape(a,1,10),reshape(b,1,10),q0];
fun = matlabFunction(cond(Yr),'Vars',{X});
X0 = [zeros(1,20)/10, zeros(1,2)];
Aeq = [];
beq = [];
lb = [];
ub = [];
A1 = [];
A2 = [];
b = [];
for i=1:jnt_num
    for j=1:Tf/Ts
        ma = [];
        mb = [];
        vma = [];
        vmb = [];
        for k=1:fourier_num
            tmp_ma = sin(wf*k*j*Ts)/(wf*k);
            tmp_mb = -cos(wf*k*j*Ts)/(wf*k);
            tmp_vma = cos(wf*k*j*Ts);
            tmp_vmb = sin(wf*k*j*Ts);
            ma = [ma,tmp_ma];
            mb = [mb,tmp_mb];
            vma = [vma, tmp_vma];
            vmb = [vmb, tmp_vmb];
        end
        
        tmpA = zeros(1,22);
        tmpA(1,21:22) = 1;
        if (i==1)
            for k=1:length(ma)
                tmpA(1,2*k-1) = ma(k);
                tmpA(1,2*k+9) = mb(k);
                tmpA(2,2*k-1) = vma(k);
                tmpA(2,2*k+9) = vmb(k);
            end            
            A1 = [A1;tmpA];
        else
            for k=1:length(ma)
                tmpA(1,2*k) = ma(k);
                tmpA(1,2*k+10) = mb(k);
                tmpA(2,2*k) = vma(k);
                tmpA(2,2*k+10) = vmb(k);                
            end
            A2 = [A2;tmpA];
        end
        tmpb = [qmax(i); dqmax(i)];
        b = [b;tmpb];            
    end       
end
A = [A1;A2];

options = optimoptions('fmincon','Algorithm','active-set');
problem.options = options;
problem.solver = 'fmincon';
problem.objective = fun;
problem.x0 = X0;
[fourierval,fval] = fmincon(problem);
