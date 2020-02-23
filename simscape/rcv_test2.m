clear
clc

%配置DH参数（改进DH法）
L1 = Link('d', 0, 'a', 0, 'alpha', 0,'modified');
L2 = Link('d', 0, 'a', 1, 'alpha', 0,'modified');
L3 = Link('d',0,'a',2,'alpha',0,'modified');

bot = SerialLink([L1 L2 L3], 'name', 'double-link')

bot.fkine([0 0 0]);
j_trans = bot.jacob0([0 0 0],'trans');
j_rot = bot.jacob0([0 0 0],'rot');
jacobian = [j_trans; j_rot]
bot.rne([0 0 0],[0 0 0],[0 0 0],'gravity',[0 9.81 0])

