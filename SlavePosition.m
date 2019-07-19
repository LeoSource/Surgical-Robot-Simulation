function [ end_Pos ] = SlavePosition( jointPos )
%   此处显示有关此函数的摘要
%   此处显示详细说明

    global P01_orl P12_orl P23_orl P34_orl P45_orl P56_orl P6t_orl
    global P01 P02 P03 P04 P05 P06 P0t
    global R01 R02 R03 R04 R05 R06
    
    RotZ     =  @(theta)([cos(theta) -sin(theta) 0
                      sin(theta)  cos(theta) 0
                          0           0      1]);
    %%%%%%%%%%%%%各坐标系原点相对基坐标系原点矢量在基坐标系下表示%%%%%%%%%%%%%%%
    P01    = P01_orl;
    P02    = P01 + R01*RotZ(jointPos(1))*P12_orl;
    P03    = P02 + R02*RotZ(jointPos(2))*P23_orl;
    P04    = P03 + R03*([0 ; 0; jointPos(3)] + P34_orl);
    P05    = P04 + R04*RotZ(jointPos(4))*P45_orl;
    P06    = P05 + R05*RotZ(jointPos(5))*P56_orl;
    P0t    = P06 + R06*RotZ(jointPos(6))*P6t_orl;
    
    end_Pos = P0t;
    
end

