clc

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