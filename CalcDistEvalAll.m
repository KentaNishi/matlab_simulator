%% �������̎Z�o�i���s�҂͌��݈ʒu�𗘗p�j
function [dist_ped ind]=CalcDistEvalAll(traj,Pedestrian,dist_max)
%���s�҂Ƃ̋����]���l���v�Z����֐�
% dist_ped = dist_max;
dist_ped = 10;
ind = 1;
for i = 1:length(traj(1,:))
    for io=1:length(Pedestrian)
        disttmp(1) = Pedestrian(io).tmp_position(1)-traj(1,i);
        disttmp(2) = Pedestrian(io).tmp_position(2)-traj(2,i);
%         disttmp(io)=norm(Pedestrian(io).tmp_position-x(1:2)');%�p�X�̈ʒu�Ə�Q���Ƃ̃m�����덷���v�Z
        if norm(disttmp) < dist_ped
            dist_ped = norm(disttmp);
            ind = io;
        end
    end
end
% [dist_ob,ind] = min(disttmp);
% dist = min([dist_max, dist_ob]);