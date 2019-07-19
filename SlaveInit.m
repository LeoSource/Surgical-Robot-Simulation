function [ output ] = SlaveInit( length )
%   �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

    global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R6t_orl
    global P01_orl P12_orl P23_orl P34_orl P45_orl P56_orl P6t_orl
    
    %%%%%%%%%%%%%%%%%%%������ϵ��ʼ��ת����%%%%%%%%%%%%%%%%
    theta1   =  30*pi/180;
    %length   =  [0.16 0.7369 0 0.338 0 0.009 0];   %%������ϵԭ���ʼ��ࣺ����ϵ0ԭ�㵽����ϵ1ԭ������������ϵ0�ı�ʾ�������Դ�����
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
    
    output = length;
end

