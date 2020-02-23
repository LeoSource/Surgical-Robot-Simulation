clear
clc

syms q1 q2 dq1 dq2 ddq1 ddq2
syms l1 l2
syms G g
G = sym([0; g; 0]);
%配置DH参数（改进DH法）
L1 = Link('d', 0, 'a', 0, 'alpha', 0,'modified');
L2 = Link('d', 0, 'a', l1, 'alpha', 0,'modified');
L3 = Link('d',0,'a',l2,'alpha',0,'modified');

%配置惯性参数
L1.m = sym('m1');

bot = SerialLink([L1 L2 L3], 'name', 'double-link')

bot.fkine([q1 q2 0]);
j_trans = bot.jacob0([q1 q2 0],'trans');
j_rot = bot.jacob0([q1 q2 0],'rot');
jacobian = simplify([j_trans; j_rot])
bot.rne([q1 q2 0],[dq1 dq2 0],[ddq1 ddq2 0],'gravity',G)
