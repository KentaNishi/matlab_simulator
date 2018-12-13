# matlab_simulator
歩行者が存在する環境下でのmatlabシミュレーション
working_spaceは作業用のbranch

---観測部分--------------------------------
occupancy_matrix=
caluculate_occupanted_area(x,Pedestrian_ver3)
occupancy_matrix:LRFの極座標系での各角度を占める歩行者の番号とその距離

predict_obserbable_pedestrian_position = predict_position_for_simulator_ver3(occupancy_matrix,predict_obserbable_pedestrian_position,x,Pedestrian_ver3); 
絶対座標系での歩行者の代表位置と観測時間

---予測部分（現状では速度とマージンの考慮）-----
existence_prohability_distrubution = state_transition_model_ver11(x,predict_obserbable_pedestrian_position ,existence_prohability_distrubution,Pedestrian_ver3,i,time_pre);
三次元の時空間占有格子地図の作成

---動作計画部-------------------------------

motion_planner_for_c1_c3(x,existence_prohability_distrubution_all(:,:,:),goal,predict_obserbable_pedestrian_position,i,Pedestrian_ver3);
