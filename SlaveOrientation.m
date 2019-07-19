function [ end_Orientation ] = SlaveOrientation( jointPos )
%   �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

    
    global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R6t_orl
    global R01 R02 R03 R04 R05 R06 R0t
    
    RotZ     =  @(theta)([cos(theta) -sin(theta) 0
                          sin(theta)  cos(theta) 0
                              0           0      1]);

    %%%%%%%%%%%%%%������ϵ��Ի�����ϵ����ת����%%%%%%%%%%%%%%%%
    R01     = R01_orl;                         %%����ϵ1�������ϵ0����ת����
    R02     = R01*RotZ(jointPos(1))*R12_orl;   %%����ϵ2�������ϵ0����ת����
    R03     = R02*RotZ(jointPos(2))*R23_orl;
    R04     = R03*RotZ(0)*R34_orl;             %%�ؽ�3λ�ƶ��ؽڣ�δ��������ϵ��ת
    R05     = R04*RotZ(jointPos(4))*R45_orl;
    R06     = R05*RotZ(jointPos(5))*R56_orl;
    R0t     = R06*RotZ(jointPos(6))*R6t_orl;

    end_Orientation = R0t;

end

