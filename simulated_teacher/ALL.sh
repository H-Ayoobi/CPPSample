
## results of local HDP
mkdir LDA_results_consistant;
mkdir HDP_results_consistant;
mkdir Local_LDA_results_consistant;
mkdir Local_HDP_results_consistant;
let exp_num=0;

##roscore
roscore 2>/dev/null > /dev/null 2>&1 &
echo "roscore is running"
sleep 3

for i in `seq 1 10`; do 
	##LDA service
	rosrun race_topic_modeling_services LDA_service.py > /dev/null 2>&1 &
	echo "LDA service is running"
	sleep 10
	roslaunch rug_simulated_user simulated_user_local_HDP.launch randorder:=true name_of_approach:=LDA
	let exp_num=exp_num+1;
	mv "experiment_1" "LDA_exp$exp_num" ; 
	mv "LDA_exp$exp_num" "LDA_results_consistant" ; 
	sleep 5

	## HDP service
	rosrun race_topic_modeling_services HDP_service.py > /dev/null 2>&1 &
	echo "HDP service is running"
	sleep 10
	roslaunch rug_simulated_user simulated_user_local_HDP.launch name_of_approach:=HDP
	mv "experiment_1" "HDP_exp$exp_num" ; 
	mv "HDP_exp$exp_num" "HDP_results_consistant" ; 
	sleep 5

	##local LDA service
	rosrun race_topic_modeling_services local_LDA_service.py  > /dev/null 2>&1 &
	echo "Local_LDA service is running"
	sleep 10
	roslaunch rug_simulated_user simulated_user_local_HDP.launch name_of_approach:=Local_LDA
	mv "experiment_1" "Local_LDA_exp$exp_num" ; 
	mv "Local_LDA_exp$exp_num" "Local_LDA_results_consistant" ; 
	sleep 5

	##local HDP service
	rosrun race_topic_modeling_services local_HDP_service.py > /dev/null 2>&1 &
	echo "local HDP service is running"
	sleep 10
	roslaunch rug_simulated_user simulated_user_local_HDP.launch name_of_approach:=Local_HDP
	mv "experiment_1" "local_HDP_exp$exp_num" ; 
	mv "local_HDP_exp$exp_num" "Local_HDP_results_consistant" ; 
	sleep 5

done
 
