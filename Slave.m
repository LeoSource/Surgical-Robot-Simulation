classdef Slave
    %UNTITLED5 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        Jw  =  zeros(3,6);
        Jv  =  zeros(3,6);
    end
    
    methods
        function ParameterInit()
            
        end
        function SlaveInit()
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
        end
        function Kinematics()
            
        end
        function DiffKinematics()
            
        end
    end
    
end

