function [ output ] = SlaveInit( length )
%   此处显示有关此函数的摘要
%   此处显示详细说明

    global R01_orl R12_orl R23_orl R34_orl R45_orl R56_orl R6t_orl
    global P01_orl P12_orl P23_orl P34_orl P45_orl P56_orl P6t_orl
    
    %%%%%%%%%%%%%%%%%%%各坐标系初始旋转矩阵%%%%%%%%%%%%%%%%
    theta1   =  30*pi/180;
    %length   =  [0.16 0.7369 0 0.338 0 0.009 0];   %%各坐标系原点初始间距：坐标系0原点到坐标系1原点向量在坐标系0的表示，后续以此类推
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
    
    %%%%%%%%%%%%%%%%%%%坐标系原点位置%%%%%%%%%%%%%%%%%%%%%%
    P01_orl  =  [0 0 -length(1)]';  %%坐标系1的原点在坐标系0中的表示
    P12_orl  =  [0 0 length(2)]';
    P23_orl  =  [0 0 0]';
    P34_orl  =  [0 0 -length(4)]';
    P45_orl  =  [0 0 0]';
    P56_orl  =  [0 -length(6) 0]';
    P6t_orl  =  [0 0 0]';
    
    output = length;
end

