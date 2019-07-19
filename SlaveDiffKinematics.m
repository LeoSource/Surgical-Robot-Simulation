function [ jacobian ] = SlaveDiffKinematics( input )
%%%%%%%%%%%%%%%%%�˶�ѧ���ſ˱Ⱦ������%%%%%%%%%%%%%%%%%
%%%%%%%Ŀ������ϵAt���ٶ�/���ٶ���֮ǰ��A1~A6��������ϵ���˶����ӵõ�%%%%%%%%
%%%%%%%wt = w1 + w2 + w3 + w4 + w5 + w6
%%%%%%%vt = w1��P1t + w2��P2t + v3 + w4��P4t + w5��P5t + w6��P6t 
   
    global R01 R02 R03 R04 R05 R06
    global P01 P02 P03 P04 P05 P06 P0t
    global Jw Jv
    
    Ez       = [0 0 1]';
    Jw(:,1)  = R01*Ez;
    Jw(:,2)  = R02*Ez;
    Jw(:,3)  = zeros(3,1);%�ؽ�3Ϊ�ƶ��ؽ�
    Jw(:,4)  = R04*Ez;
    Jw(:,5)  = R05*Ez;
    Jw(:,6)  = R06*Ez;

    Jv(:,1)  = cross(Jw(:,1),(P0t - P01));
    Jv(:,2)  = cross(Jw(:,2),(P0t - P02));
    Jv(:,3)  = R03*Ez;%�ؽ�3Ϊ�ƶ��ؽ�
    Jv(:,4)  = cross(Jw(:,4),(P0t - P04));
    Jv(:,5)  = cross(Jw(:,5),(P0t - P05));
    Jv(:,6)  = cross(Jw(:,6),(P0t - P06));

    jacobian = [Jv ; Jw];


end

