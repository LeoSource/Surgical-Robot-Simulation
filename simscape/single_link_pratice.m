%%���������������ʶ��ȷ
g = 9.8;
l = 0.5;
start_num = 500;
Y = [];
for i=start_num:length(tout)
    Y1 = 1/4*l^2*q(i) + 1/2*g*l*cos(q(i));
    Y2 = ddq(i);
    tmpY = [Y1 Y2];
    Y = [Y; tmpY];
end
%p=[m;I]�����������
p = inv(Y'*Y)*Y'*tau(start_num:end)