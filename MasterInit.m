function [ output ] = MasterInit( g_length )
% 此处显示有关此函数的摘要
%   此处显示详细说明

    global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R67_orl R78_orl R89_orl R9t_orl
    global P01_orl P12_orl P23_orl P34_orl P45_orl

    %%%%%%%%%%%%%%%%%%%各坐标系初始旋转矩阵%%%%%%%%%%%%%%%%
    %theta1   =  30*pi/180;
    %length   =  [0.16 0.7369 0 0.338 0 0.009 0];   %%各坐标系原点初始间距：坐标系0原点到坐标系1原点向量在坐标系0的表示，后续以此类推
    R01_orl  =  eye(3);
    R12_orl  =  [ 0 1 0; 0 0 1; 1 0 0 ];
    R23_orl  =  eye(3);
    R34_orl  =  [ 0 0 1; 1 0 0; 0 1 0 ];
    R45_orl  =  eye(3);
    R56_orl       =  [ 0 1 0; 0 0 1; 1 0 0];
    R67_orl       =  [ 0 0 1; 1 0 0; 0 1 0];
    R78_orl       =  [ 0 0 1; 1 0 0; 0 1 0];
    R89_orl       =  [ 0 1 0; 0 0 1; 1 0 0];
    R9t_orl       =  [ 0 0 1; -1 0 0; 0 -1 0];  %视野坐标系至A9坐标系的转换矩阵
    
    %%%%%%%%%%%%%%%%%%%坐标系原点位置%%%%%%%%%%%%%%%%%%%%%%
    P01_orl       =  [ 0 0 -g_length(1)]';
    P12_orl       =  [ 0 0 -g_length(2)]';
    P23_orl       =  [ 0 g_length(3) 0]';
    P34_orl       =  [ -g_length(4) 0 0]';
    P45_orl       =  [ 0 g_length(5) g_length(6)]';%%A5/A6/A7/A8坐标系原点重合
    
    output = g_length;

end

