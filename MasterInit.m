function [ output ] = MasterInit( g_length )
% �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

    global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R67_orl R78_orl R89_orl R9t_orl
    global P01_orl P12_orl P23_orl P34_orl P45_orl

    %%%%%%%%%%%%%%%%%%%������ϵ��ʼ��ת����%%%%%%%%%%%%%%%%
    %theta1   =  30*pi/180;
    %length   =  [0.16 0.7369 0 0.338 0 0.009 0];   %%������ϵԭ���ʼ��ࣺ����ϵ0ԭ�㵽����ϵ1ԭ������������ϵ0�ı�ʾ�������Դ�����
    R01_orl  =  eye(3);
    R12_orl  =  [ 0 1 0; 0 0 1; 1 0 0 ];
    R23_orl  =  eye(3);
    R34_orl  =  [ 0 0 1; 1 0 0; 0 1 0 ];
    R45_orl  =  eye(3);
    R56_orl       =  [ 0 1 0; 0 0 1; 1 0 0];
    R67_orl       =  [ 0 0 1; 1 0 0; 0 1 0];
    R78_orl       =  [ 0 0 1; 1 0 0; 0 1 0];
    R89_orl       =  [ 0 1 0; 0 0 1; 1 0 0];
    R9t_orl       =  [ 0 0 1; -1 0 0; 0 -1 0];  %��Ұ����ϵ��A9����ϵ��ת������
    
    %%%%%%%%%%%%%%%%%%%����ϵԭ��λ��%%%%%%%%%%%%%%%%%%%%%%
    P01_orl       =  [ 0 0 -g_length(1)]';
    P12_orl       =  [ 0 0 -g_length(2)]';
    P23_orl       =  [ 0 g_length(3) 0]';
    P34_orl       =  [ -g_length(4) 0 0]';
    P45_orl       =  [ 0 g_length(5) g_length(6)]';%%A5/A6/A7/A8����ϵԭ���غ�
    
    output = g_length;

end

