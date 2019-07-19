function [ m_Rot ] = MasterFollowRotation( slave_setjointPos,slave_shoulderjointPos,endo_setjoitPos,endo_shoulderjointPos,endo_jointPos,s_Rot )
%MasterFollowRotation: return master expect rotation
%   此处显示详细说明

    endo_Length = [0.16 0.7369 0.278 0 0 0];    %内窥镜臂杆长参数
    endoscopeDegree = 30;                       %内窥镜角度

%     slave_setjointPos = [ 0 0 0 0 ];            %从手调整臂关节角
%     slave_shoulderjointPos = 0;                 %从手子悬吊关节角
%     endo_setjoitPos = [ 0 0 0 0 ];              %内窥镜调整臂关节角
%     endo_shoulderjointPos = 0;                  %内窥镜子悬吊关节角
    
    Ti_Rot_Ei = rotz(-(slave_setjointPos(1) + slave_setjointPos(4) + slave_shoulderjointPos) + ...
                       endo_setjoitPos(1) + endo_setjoitPos(4) + endo_shoulderjointPos);
    nothing = SlaveInit(endo_Length);
    endo_CartRot_tmp = SlaveOrientation(endo_jointPos);
    endo_CartRot = endo_CartRot_tmp * rotx(endoscopeDegree);
    Ti_Rot_E = Ti_Rot_Ei * endo_CartRot;
    
    m_Rot = inv(Ti_Rot_E) * s_Rot;                 %E_Rot_T
    
end

