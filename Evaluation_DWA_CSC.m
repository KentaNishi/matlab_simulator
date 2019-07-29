function [evalDB,trajDB]=Evaluation_DWA_CSC(x,Vr,goal,Kinematic,PARAM,Pedestrian)

global vehicle_width;
global vehicle_front_length;
global vehicle_rear_length;
global wall_left;
global wall_right;
evalDB=[];
trajDB=[];
p_ped =zeros(2,length(Pedestrian));
car_y_position = zeros(4,1);
for vt=Vr(1):Kinematic(5):Vr(2) % vt=velocity
    for ot=Vr(3):Kinematic(6):Vr(4) % ot=steer angle
        %ãOê’ÇÃêÑíË
        [xt,traj]=GenerateTrajectory(x,vt,ot,PARAM(6),Kinematic);
        %äeï]âøä÷êîÇÃåvéZ        
        
        heading = CalcHeadingEval(xt,goal);%ê≥ãKâªÇÕÇµÇƒÇ»Ç¢ÅCÇ†Ç∆Ç≈Ç‹Ç∆ÇﬂÇƒê≥ãKâª
        vel = CalcVelEval(vt);%ê≥ãKâªÇÕÇµÇƒÇ»Ç¢ÅCÇ†Ç∆Ç≈Ç‹Ç∆ÇﬂÇƒê≥ãKâª
        for i = 1:length(Pedestrian)
            p_ped(:,i) = [Pedestrian(i).tmp_position(1);Pedestrian(i).tmp_position(2)];
        end
        car_y_position(1) = xt(2)+vehicle_width/2*cos(xt(3))+vehicle_front_length*sin(xt(3));
        car_y_position(2) = xt(2)-vehicle_width/2*cos(xt(3))+vehicle_front_length*sin(xt(3));
        car_y_position(3) = xt(2)+vehicle_width/2*cos(xt(3))-vehicle_rear_length*sin(xt(3));
        car_y_position(4) = xt(2)-vehicle_width/2*cos(xt(3))-vehicle_rear_length*sin(xt(3));
        p_m_dist = p_ped - xt(1:2);
        ob_dist = min([abs(wall_left-car_y_position);abs(wall_right-car_y_position)]);
        ped_dist =  sqrt(min(sum(p_m_dist.*p_m_dist)));
        dist = min(ob_dist,ped_dist);
        dir_sp = calc_dir_sp(x,Pedestrian,xt);
        vel_sp = calc_vel_sp(x,Pedestrian,xt);
        
        
        
        %ëÄçÏâ¬î\ë¨ìx
        possible_value = possible_velocity([vt ot]);
        
        %à¿ëSê´ÇÃîªíË(0 or 1, 0=danger)     
        safe_wall = CalcWallCheck(traj);
        safe_ped_static = Judgement_margin_2_time(traj, Pedestrian);
        %safe_ped_crossing = Judgement_crossing_3_2(x,[vt ot],Pedestrian);
        %safe_ped = min(safe_ped_static,safe_ped_crossing);
        
%         if ~safe_ped
%             disp('danger with ped')
%         end
        %collision_safety = min(safe_wall,safe_ped);
        collision_safety = min(safe_wall,safe_ped_static);
        safety = min(possible_value,collision_safety);
        if safety == 0
            evalDB = [evalDB;[vt ot 0,0,0,0,0]];
        else
            EVAL = [heading,dist,vel,dir_sp,vel_sp];
            evalDB=[evalDB;[vt ot EVAL]];
        end
        trajDB=[trajDB;traj];     
    end
end