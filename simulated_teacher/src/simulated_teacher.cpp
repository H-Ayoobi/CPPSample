// ############################################################################
//    
//   Created: 	2020
//   Author : 	
//   Email  :	
//   Purpose:  This program follows  the teaching protocol and autonomously interact with 
//				the agent using teach, ask and correct actions. For each newly taught
//				category, the average sucess of the system should be computed. To do that, 
//				the simulated teacher repeatedly picks object views of the currently
//				known categories from a database and presents them to the system for 
//				checking whether the system can recognize them. If not, the simulated
//				teacher provides corrective feedback.
//   		
// 
// ############################################################################

/* _________________________________
|                                 |
|             INCLUDES            |
|_________________________________| */
  
//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

//ROS & PCL includes 
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

//pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

//package includes
#include <backbone/spin_image.h>
#include <backbone/perception_msgs.h>
#include <backbone/perception_db.h>
#include <backbone/perception_db_serializer.h>
#include <backbone/print.h>
#include <simulated_teacher/simulated_teacher_functionality.h>

// define topic modelling service here
#include <topic_modeling_services/topic_modelling.h>


/* _______________________________
|                                 |
|            NameSpace            |
|_________________________________| */

using namespace pcl;
using namespace std;
using namespace ros;
using namespace backbone;
typedef pcl::PointXYZRGBA PointT;

/* _______________________________
|                                 |
|        Global Parameters        |
|_________________________________| */

std::string name_of_approach = "Local_HDP";

//dataset
// string home_address = "/media/cor/4804-C7CA/washington/"; 
string home_address = "";
string dictionary_address = "";

//simulated user parameters
double protocol_threshold = 0.67;  
int user_sees_no_improvment_const = 100;
int window_size = 3;
int number_of_categories =51 ; 
double uniform_sampling_size = 0.03;
double voxel_size_for_keypoints = 0.01;

//spin images parameters
int    spin_image_width = 8;
double spin_image_support_lenght = 0.06;
int    subsample_spinimages = 10;
double recognition_threshold = 100000;  
double keypoint_sampling_size = 0.01;

bool downsampling = false;
double downsampling_voxel_size = 0.01;

int dictionary_size = 90;
vector <SITOV> dictionary;

bool randorder = false;

// because of updating memory in evaluation fuction, topic_modelling_server should be gloabal.
ros::ServiceClient topic_modelling_server;
std::string topic_modeling_service = "/local_HDP_service";

/* _______________________________
|                                 |
|         Global Variable         |
|_________________________________| */

PerceptionDB* _pdb;
typedef pcl::PointXYZRGBA PointT;

unsigned int cat_id = 1;
unsigned int track_id =1;
unsigned int view_id = 1;
string InstancePathTmp= "";

int  off_line_flag  = 1;
int  number_of_bins = 5;
int  adaptive_support_lenght = 1;
double global_image_width =0.5;
int sign = 1;
int threshold = 10;	 
std::string evaluation_file, evaluationTable, precision_file, local_f1_vs_learned_category, f1_vs_learned_category;
std::ofstream summary_of_experiment , PrecisionMonitor, local_f1_vs_category, f1_vs_category, NumberofFeedback , category_random, instances_random, category_introduced;
int TP = 0, FP= 0, FN = 0, tp_tmp = 0, fp_tmp = 0, fn_tmp = 0, obj_num = 0, number_of_instances = 0;
vector <int> recognition_results; // we coded 0: continue(correctly detect unkown object)
								// 1: TP , 2: FP , 3: FN , 4: FP and FN
			

///////////////////////////////////////////////////////////////////////////////////////////////////
void evaluationFunction(const backbone::RRTOV &result)
{
    PrettyPrint pp;
    ROS_INFO("ground_truth_name = %s", result.ground_truth_name.c_str());
    string instance_path = result.ground_truth_name.substr(home_address.size(),result.ground_truth_name.size());
	string tmp = instance_path;
	string true_cat = extractCategoryName(tmp);

    std:: string object_name;
    object_name = extractObjectNameSimulatedUser (result.ground_truth_name);
    pp.info(std::ostringstream().flush() << "extractObjectName: " << object_name.c_str()); 
    
    obj_num++;
    pp.info(std::ostringstream().flush() << "track_id="<< result.track_id << "\tview_id=" << result.view_id);
          
    float minimum_distance = result.minimum_distance;
 
    string predicted_cat;
    predicted_cat= result.recognition_result.c_str();   
    pp.info(std::ostringstream().flush() << "[-]object_name: "<< object_name.c_str());
    pp.info(std::ostringstream().flush() << "[-]true_category: "<<true_cat.c_str());
    pp.info(std::ostringstream().flush() << "[-]predicted_category: " << predicted_cat.c_str());
	true_cat = fixedLength (true_cat , 15); // to have a pretty report, while the length of given string is less than 15, we add space at the end of the string
	predicted_cat = fixedLength (predicted_cat , 15); // to have a pretty report, while the length of given string is less than 15, we add space at the end of the string

    char unknown[] = "unknown";
   
    summary_of_experiment.open( evaluation_file.c_str(), std::ofstream::app);    
    summary_of_experiment.precision(3);
   
    if ((strcmp(true_cat.c_str(),unknown)==0) && (strcmp(predicted_cat.c_str(),unknown)==0))
    { 	
		recognition_results.push_back(0);// continue
		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "0\t0\t0"<< "\t\t"<< minimum_distance;
		summary_of_experiment << "\n----------------------------------------------------------------------------------------------------------------------------------------";
    }
    else if ((strcmp(true_cat.c_str(),predicted_cat.c_str())==0))
    { 
		TP++;
		tp_tmp++;
		recognition_results.push_back(1);
		
		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "1\t0\t0" << "\t\t"<< minimum_distance;
		summary_of_experiment << "\n----------------------------------------------------------------------------------------------------------------------------------------";
    }
    else if ((strcmp(true_cat.c_str(),unknown)==0) && (strcmp(predicted_cat.c_str(),unknown)!=0))
    { 	
		FP++; 
		fp_tmp++;
		recognition_results.push_back(2);

		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "0\t1\t0"<< "\t\t"<< minimum_distance;
		summary_of_experiment << "\n----------------------------------------------------------------------------------------------------------------------------------------";
    }
    else if ((strcmp(true_cat.c_str(),unknown)!=0) && (strcmp(predicted_cat.c_str(),unknown)==0))
    { 	
		FN++;
		fn_tmp++;
		recognition_results.push_back(3);

		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "0\t0\t1"<< "\t\t"<< minimum_distance;
		summary_of_experiment << "\n========================================================================================================================================";

		number_of_instances ++;
	
		introduceNewInstanceLocalHDP ( home_address,
										instance_path,
										cat_id, track_id, view_id,
										spin_image_width,
										spin_image_support_lenght,
										voxel_size_for_keypoints,
										dictionary,
										topic_modelling_server,
										pp );
		
		track_id++;
		pp.info(std::ostringstream().flush() << "[-]Category Updated");
    }
    else if ((strcmp(true_cat.c_str(),predicted_cat.c_str())!=0))
    {  	
		FP++; FN++;
		fp_tmp++; fn_tmp++;    
		recognition_results.push_back(4);

		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "0\t1\t1"<< "\t\t"<< minimum_distance;
		summary_of_experiment << "\n========================================================================================================================================";

		number_of_instances++;

		
		introduceNewInstanceLocalHDP ( home_address,
									instance_path,
									cat_id, track_id, view_id,
									spin_image_width,
									spin_image_support_lenght,
									voxel_size_for_keypoints,
									dictionary,
									topic_modelling_server,
									pp );


		track_id++;
		pp.info(std::ostringstream().flush() << "[-]Category Updated");
    }
    summary_of_experiment.close();
    summary_of_experiment.clear();
    pp.printCallback();
}

int main(int argc, char** argv)
{
   
	/* __________________________________
	|                                   |
	|  Creating a folder for each RUN   |
	|___________________________________| */
	int experiment_number = 1;
	string system_command= "mkdir "+ ros::package::getPath("simulated_teacher")+ "/result/experiment_1";
	system( system_command.c_str());

	PrettyPrint pp; // pp stands for pretty print


	precision_file = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/PrecisionMonitor.txt";
	PrecisionMonitor.open (precision_file.c_str(), std::ofstream::trunc);
	PrecisionMonitor.precision(4);
	PrecisionMonitor.close();

	local_f1_vs_learned_category = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/local_f1_vs_learned_category.txt";
	local_f1_vs_category.open (local_f1_vs_learned_category.c_str(), std::ofstream::trunc);
	local_f1_vs_category.precision(4);
	local_f1_vs_category.close();
	
	f1_vs_learned_category = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/f1_vs_learned_category.txt";
	f1_vs_category.open (f1_vs_learned_category.c_str(), std::ofstream::trunc);
	f1_vs_category.precision(4);
	f1_vs_category.close();
		
	ros::init (argc, argv, "EVALUATION");
	ros::NodeHandle nh;
	
	// initialize perception database 
	_pdb = backbone::PerceptionDB::getPerceptionDB(&nh); //initialize the database class_list_macros
	string name = nh.getNamespace();
	
	/* _____________________________________
	|                                       |
	|    read prameters from launch file    |
	|_______________________________________| */

	
	// read database parameter
	nh.param<std::string>("/perception/home_address", home_address, home_address);
	nh.param<int>("/perception/number_of_categories", number_of_categories, number_of_categories);
	nh.param<std::string>("/perception/name_of_approach", name_of_approach, name_of_approach);
	set_home_address(home_address);

	// read dictionary_address
	nh.param<std::string>("/perception/dictionary_address", dictionary_address, dictionary_address);
	nh.param<int>("/perception/dictionary_size", dictionary_size, dictionary_size);

	///read spin images parameters
	nh.param<int>("/perception/spin_image_width", spin_image_width, spin_image_width);
	nh.param<double>("/perception/spin_image_support_lenght", spin_image_support_lenght, spin_image_support_lenght);
	nh.param<int>("/perception/subsample_spinimages", subsample_spinimages, subsample_spinimages);
	nh.param<double>("/perception/voxel_size_for_keypoints", voxel_size_for_keypoints, voxel_size_for_keypoints);

    // read topic modeling algorithm 
    nh.param<std::string>("/perception/topic_modeling_service", topic_modeling_service, topic_modeling_service);

 	//recognition threshold
    nh.param<double>("/perception/recognition_threshold", recognition_threshold, recognition_threshold);

    // read downsampling parameter 0 = FLASE, 1 = TRUE 
    nh.param<bool>("/perception/downsampling", downsampling, downsampling);
    string downsampling_flag = (downsampling == 0) ? "FALSE" : "TRUE";
	// read downsampling_voxel_size parameter
    nh.param<double>("/perception/downsampling_voxel_size", downsampling_voxel_size, downsampling_voxel_size);
    downsampling_voxel_size = downsampling_voxel_size;


	//read simulated teacher parameters
	nh.param<double>("/perception/protocol_threshold", protocol_threshold, protocol_threshold);
	nh.param<int>("/perception/user_sees_no_improvment_const", user_sees_no_improvment_const, user_sees_no_improvment_const);
	nh.param<int>("/perception/window_size", window_size, window_size);	
  	nh.param<bool>("/perception/randorder", randorder, randorder);	

	//recognition threshold
	nh.param<double>("/perception/recognition_threshold", recognition_threshold, recognition_threshold);

	string dataset= (home_address == "/home/hamidreza/") ? "Restaurant Object Dataset" : "RGB-D Washington";

	evaluation_file = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/summary_of_experiment.txt";
	summary_of_experiment.open (evaluation_file.c_str(), std::ofstream::out);
	
	summary_of_experiment  <<"system configuration:" 
			<< "\n\t-experiment_name = " << name_of_approach
			<< "\n\t-name_of_dataset = " << dataset //TODO make a parameter for this 
			<< "\n\t-spin_image_width = "<<spin_image_width 
			<< "\n\t-spin_image_support_lenght = "<< spin_image_support_lenght
			<< "\n\t-dictionary_size = "<< dictionary_size	
			<< "\n\t-voxel_size = " << voxel_size_for_keypoints
			// << "\n\t-downsampling = " << downsampling_flag
			// << "\n\t-downsampling_voxel_size = " << downsampling_voxel_size
			<< "\n\t-simulated_user_threshold = " << protocol_threshold 			
			<< "\n------------------------------------------------------------------------------------------------------------------------------------\n\n";

	summary_of_experiment << "\n\nNo."<<"\tobject_name" <<"\t\t\t\t"<< "ground_truth" <<"\t\t"<< "prediction"<< "\t\t"<< "TP" << "\t"<< "FP"<< "\t"<< "FN \t\tdistance";
	summary_of_experiment << "\n-----------------------------------------------------------------------------------------------------------------------------------------";
	summary_of_experiment.close();


	ROS_INFO ("home address = %s", home_address.c_str());
	/* _______________________________
	|                                 |
	|     Randomly sort categories    |
	|_________________________________| */
	
	if(randorder){
 		generateRrandomSequencesCategories(experiment_number);
	}

	string category_introduced_txt = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/Category_Introduced.txt";
	category_introduced.open (category_introduced_txt.c_str(), std::ofstream::out);

	/* _________________________________
	|                                   |
	|  read dictionary of visual words  |
	|___________________________________| */
	
	///read the initial dictionary of visual words
	char buffer [500];
	int spin_image_support_lenght_float_tmp = spin_image_support_lenght * 100;
	double m=sprintf (buffer, "%s%i%i%i%s","clusters", dictionary_size, spin_image_width, spin_image_support_lenght_float_tmp,".txt");
	string dictionary_name= buffer; 
	ROS_INFO ("dictionary_name = %s", dictionary_name.c_str());
	string IEETA_or_Washington = (home_address == "/home/hamidreza/") ? "IEETA/" : "Washington/";
	// string dictionary_path = ros::package::getPath("race_backbone_evaluation")+ "/generic_specific_dictionaries/" 
	// 				+ IEETA_or_Washington.c_str() + dictionary_name.c_str();

	string dictionary_path = dictionary_address;
	// string dictionary_path = "/home/cor/wash-clusters90810.txt" ;

	dictionary = readClusterCenterFromFile (dictionary_path);
	ROS_INFO ("dictionary_path = %s", dictionary_path.c_str());
	ROS_INFO ("initial_dictionary.size() = %ld", dictionary.size());

	// /// NOTE: read diffrent dictionary based on the file config and use for a set of expriments
	// string dictionary_path = ros::package::getPath("race_object_representation") + "/clusters.txt";
	// ROS_INFO ("dictionary_path = %s", dictionary_path.c_str());
	// vector <SITOV> dictionary = readClusterCenterFromFile (dictionary_path);    
	// ROS_INFO ("dictionary_size = %d", dictionary.size());

	/* _____________________________
    |                               |
    |     create a client server    |
    |_______________________________| */
    topic_modelling_server = nh.serviceClient<topic_modeling_services::topic_modelling>(topic_modeling_service); // /local_HDP_server -> param like deep_learning_architecture
    pp.info(std::ostringstream().flush() <<"local_HDP based simulated user -> Hello World");

	/* _______________________________
	|                                |
	|         Initialization         |
	|________________________________| */

	string instance_path= "";	
	int class_index = 1; // class index
	unsigned int instance_number = 1;
	cat_id = 1;  // it is a constant to create a key for each category <Cat_Name><Cat_ID>
	track_id = 1;
	view_id = 1; // it is a constant to create key for categories <TID><VID> (since the database samples were
		         // collected manually, each track_id has exactly one view)

	vector <unsigned int> instance_number2;
	for (int i = 0; i < number_of_categories ; i++)
	{
	    instance_number2.push_back(1);
	}
	    
	int number_of_taught_categories = 0;

	
	//start tic	
	ros::Time begin_process = ros::Time::now(); 
	ros::Time start_time = ros::Time::now();
	
	/* ______________________________
	|                                |
	|        Introduce category      |
	|________________________________| */
	
	introduceNewCategoryLocalHDP(	home_address, 
									class_index, track_id, instance_number2.at(class_index-1),
									evaluation_file,
									spin_image_width,
									spin_image_support_lenght,
									voxel_size_for_keypoints,
									dictionary,
									topic_modelling_server,
									pp);
									
	number_of_instances += 3; // we use three instances to initialize a category
	number_of_taught_categories ++;
	category_introduced << "1\n";
	
	vector <ObjectCategory> list_of_object_category = _pdb->getAllObjectCat();

	/* ______________________________
	|                                |
	|        Simulated Teacher       |
	|________________________________| */
	
	float precision = 0;
	float recall = 0;
	float f1 =0;
	vector <float> average_class_precision;
	
	while ( class_index < number_of_categories)  // one category is already taught above
	{
	    class_index ++; // class index
	    instance_path = "";
      
	    if ( introduceNewCategoryLocalHDP(	home_address, 
											class_index, track_id, instance_number2.at(class_index-1),
											evaluation_file,
											spin_image_width,
											spin_image_support_lenght,
											voxel_size_for_keypoints,
											dictionary,
											topic_modelling_server,
											pp) == -1)  
	    {
			ROS_INFO ("Note: the experiment is terminated because there is not enough test data to continue the evaluation");
			ros::Duration duration = ros::Time::now() - begin_process;
			
			reportCurrentResults( TP, FP, FN, evaluation_file, true);

			reportExperimentResult( average_class_precision,
									number_of_instances, 
									number_of_taught_categories,  
									evaluation_file, duration);
						
			reportAllExperimentalResults( TP, FP, FN, obj_num,			    
											average_class_precision,
											number_of_instances, 
											number_of_taught_categories,
											name_of_approach );

			category_introduced.close();
			
			monitorF1VsLearnedCategory( f1_vs_learned_category, TP, FP, FN);

			plotSimulatedTeacherProgressInMatlab( experiment_number, protocol_threshold, precision_file);
			//plotLocalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, protocol_threshold, local_f1_vs_learned_category.c_str());
			plotGlobalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, f1_vs_learned_category.c_str());
			plotNumberOfLearnedCategoriesVsIterationsInMatlab( experiment_number, category_introduced_txt.c_str());
			plotNumberOfStoredInstancesPerCategoryInMatlab( list_of_object_category);

			// system_command= "cp "+ home_address+ "/Category/Category.txt " + ros::package::getPath("simulated_teacher") + "/result/experiment_1" ;
			// system( system_command.c_str());
			return (0) ;
	    }

	    number_of_instances += 3; // we use three instances to initialize a category
	    number_of_taught_categories ++;
	    category_introduced << "1\n";
	    tp_tmp = 0; fp_tmp = 0; fn_tmp = 0;
	    
	    int k = 0; // number of classification results
	    float precision_tmp = 0;
	    precision = 0;
	    f1 = 0;
	    recall = 0;
	    unsigned int c = 1; // class index
	    int iterations = 1;
	    int iterations_user_sees_no_improvment = 0;
	    bool user_sees_no_improvement = false; // In the current implementation, If the simulated teacher 
															// sees the precision doesn't improve in 100 iteration, then, 
															// it terminares evaluation of the system, originally, 
															// it was an empirical decision of the human instructor
	    while ( ((f1 < protocol_threshold ) or (k < number_of_taught_categories)) and 
				(!user_sees_no_improvement) )
	    {
			category_introduced<< "0\n";
			ROS_INFO("\t\t[-] Iteration:%i",iterations);
			ROS_INFO("\t\t[-] c:%i",c);
			ROS_INFO("\t\t[-] Instance number:%i",instance_number2.at(c-1));
			//info for debug
			ROS_INFO("\t\t[-] Home address parameter : %s", home_address.c_str());
			ROS_INFO("\t\t[-] number_of_categories : %i", number_of_categories);
			ROS_INFO("\t\t[-] protocol_threshold : %lf", protocol_threshold);
			ROS_INFO("\t\t[-] user_sees_no_improvment_const : %i", user_sees_no_improvment_const);
			ROS_INFO("\t\t[-] window_size : %i", window_size);

			// select an instance from an specific category
			instance_path= "";
			selectAnInstancefromSpecificCategory(c, instance_number2.at(c-1), instance_path);
			ROS_INFO("\t\t[-]-Test Instance: %s", instance_path.c_str());
		
			// check the selected instance exist or not? 
			if (instance_path.size() < 2) 
			{
				ROS_INFO("\t\t[-]-The %s file does not exist", instance_path.c_str());
				ROS_INFO("\t\t[-]- number of taught categories= %i", number_of_taught_categories); 
				ROS_INFO("Note: the experiment is terminated because there is not enough test data to continue the evaluation");
				category_introduced.close();

				ros::Duration duration = ros::Time::now() - begin_process;
			
				reportExperimentResult( average_class_precision,
										  number_of_instances, 
										  number_of_taught_categories,  
										  evaluation_file, duration);		    		    
				
				reportCurrentResults( TP, FP, FN, evaluation_file, true);
	
				reportAllExperimentalResults( TP, FP, FN, obj_num,			    
											  average_class_precision,
											  number_of_instances, 
											  number_of_taught_categories,
											  name_of_approach);
		
				monitorF1VsLearnedCategory( f1_vs_learned_category, TP, FP, FN);
				
				plotSimulatedTeacherProgressInMatlab( experiment_number, protocol_threshold, precision_file);
				plotLocalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, protocol_threshold, local_f1_vs_learned_category.c_str());
				plotGlobalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, f1_vs_learned_category.c_str());
				plotNumberOfLearnedCategoriesVsIterationsInMatlab( experiment_number, category_introduced_txt.c_str());
				plotNumberOfStoredInstancesPerCategoryInMatlab( list_of_object_category);


				// system_command = "cp " + home_address + "/Category/Category.txt " + ros::package::getPath("simulated_teacher") + "/result/experiment_1";
				// system( system_command.c_str());
				return (0) ;	    
			}
			else
			{
				std::string ground_truth_category_name = extractCategoryName(instance_path);
				instance_path = home_address + "/" + instance_path.c_str();
				
				//load an instance from file
				boost::shared_ptr<PointCloud<PointT> > target_pc_original (new PointCloud<PointT>);
				if (io::loadPCDFile <PointXYZRGBA> (instance_path.c_str(), *target_pc_original) == -1)
				{	
					ROS_ERROR("\t\t[-]-Could not read given object %s :", instance_path.c_str());
					return(0);
				}		   
				ROS_INFO("\t\t[-]-  track_id: %i , \tview_id: %i ", track_id, view_id );
				
				/* ___________________________
				|            	 			  |
				|    Object Representation    |
				|_____________________________| */

				boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
				//// downsampling 
				pcl::VoxelGrid<PointT > voxelized_point_cloud;	
				voxelized_point_cloud.setInputCloud (target_pc_original);
				voxelized_point_cloud.setLeafSize (0.005, 0.005, 0.005);
				voxelized_point_cloud.filter (*target_pc);
				ROS_INFO( "The size of converted point cloud  = %d ", target_pc->points.size() );
				
				//Declare a boost share ptr to the spin image msg		  
				boost::shared_ptr< vector <SITOV> > object_view_spin_images;
				object_view_spin_images = (boost::shared_ptr< vector <SITOV> >) new (vector <SITOV>);
				
				boost::shared_ptr<PointCloud<PointT> > uniform_keypoints (new PointCloud<PointT>);
				boost::shared_ptr<pcl::PointCloud<int> > uniform_sampling_indices (new PointCloud<int>);
				keypointSelection( target_pc, 
									voxel_size_for_keypoints,
									uniform_keypoints,
									uniform_sampling_indices);

				ROS_INFO ("number of keypoints = %i", uniform_keypoints->points.size());
				
				
				if (!estimateSpinImages(target_pc, 
										0.01 /*downsampling_voxel_size*/, 
										0.05 /*normal_estimation_radius*/,
										spin_image_width /*spin_image_width*/,
										0.0 /*spin_image_cos_angle*/,
										1 /*spin_image_minimum_neighbor_density*/,
										spin_image_support_lenght /*spin_image_support_lenght*/,
										object_view_spin_images,
										uniform_sampling_indices /*subsample spinimages*/))
				{
					ROS_INFO("Could not compute spin images");
					continue;
				}
				
				// represent object based on BOW
				SITOV object_representation;
				objectRepresentationBagOfWords (dictionary, *object_view_spin_images, object_representation);
	
				ros::Time start_time_recognition = ros::Time::now();

				SITOV local_HDP_object_representation;
				/// call topic_modelling (local HDP) service to represent the given object
				topic_modeling_services::topic_modelling srv;
				srv.request.bag_of_words_representation = object_representation.spin_image;
				string ground_truth_label = instance_path.substr(home_address.size(),instance_path.size());
				srv.request.ground_truth_label = extractCategoryName(ground_truth_label);
				srv.request.teach_data = false;

				if (topic_modelling_server.call(srv))
				{
					for (size_t i = 0; i < srv.response.topic_representation.size(); i++)
					{
						local_HDP_object_representation.spin_image.push_back(srv.response.topic_representation.at(i));
					}
					ROS_INFO("\t\t[-] recognition using local_HDP_object_representation %s", srv.response.recognition_result.c_str());

				}
				else
				{
					ROS_ERROR("Failed to call Local_HDP service ");
					continue;
				}

				ROS_INFO("\t\t[-] Size of local_HDP_object_representation is %d", local_HDP_object_representation.spin_image.size());
				
				std::string result_string = srv.response.recognition_result.c_str();

				/* ______________________
				|                        |
				|   Object Recognition   |
				|________________________| */
			
				//// get list of all object categories
				list_of_object_category = _pdb->getAllObjectCat();
				ROS_INFO(" %d categories exist in the perception database ", list_of_object_category.size() );
				ROS_INFO("****========***** Object recognition process took = %f", (ros::Time::now() - start_time_recognition).toSec());

				//// RRTOV stands for Recognition Result of Track Object View - it is a msg defined in backbone
				RRTOV rrtov;
				rrtov.header.stamp = ros::Time::now();
				rrtov.track_id = track_id;
				rrtov.view_id = view_id;
				rrtov.recognition_result = result_string;
				rrtov.ground_truth_name = instance_path.c_str();
				
				evaluationFunction(rrtov);
			
				if (c >= number_of_taught_categories)
				{
					c = 1;
				}
				else
				{
					c++;
				}
				
				if ( (iterations >= number_of_taught_categories) and (iterations <= window_size * number_of_taught_categories))
				{    
					if (( tp_tmp + fp_tmp) != 0)
					{
						precision = tp_tmp / double (tp_tmp+fp_tmp);
					}
					else
					{
						precision = 0;
					}		
					if ((tp_tmp + fn_tmp) != 0)
					{
						recall = tp_tmp / double (tp_tmp+fn_tmp);
					}
					else
					{
						recall = 0;
					}
					if ((precision + recall) != 0)
					{
						f1 = 2 * (precision * recall) / (precision + recall);
					}
					else
					{
						f1 = 0;
					}
						
					monitorPrecision (precision_file, f1);		

					if (f1 > protocol_threshold)
					{
						average_class_precision.push_back(f1);
						user_sees_no_improvement = false;
						ROS_INFO("\t\t[-]- precision= %f", precision);
						ROS_INFO("\t\t[-]- f1 = %f", f1); 
						reportCurrentResults(tp_tmp, fp_tmp, fn_tmp, evaluation_file, false);
						iterations = 1;
						monitorPrecision (local_f1_vs_learned_category, f1);
						ros::spinOnce();		
					}  
						
				}//if
				else if ( (iterations > window_size * number_of_taught_categories)) // In this condition, if we are at iteration I>3n, we only
																					// compute precision as the average of last 3n, and discart the first
																					// I-3n iterations.
				{
					//compute f1 of last 3n, and discart the first I-3n iterations
					f1 = computeF1OfLast3n (recognition_results, number_of_taught_categories);
					// ROS_INFO("\t\t[-]- precision= %f", precision);
					monitorPrecision (precision_file, f1);		
					reportF1OfLast3n (evaluation_file, f1);

					if (f1 > protocol_threshold)  
					{
						average_class_precision.push_back(f1);
						user_sees_no_improvement = false;
						reportCurrentResults(tp_tmp, fp_tmp, fn_tmp, evaluation_file, false);
						monitorPrecision (local_f1_vs_learned_category, f1);
						iterations = 1;
						iterations_user_sees_no_improvment = 0;
						ros::spinOnce();		
					} 
					else 
					{
						iterations_user_sees_no_improvment ++;
						ROS_INFO("\t\t[-]- %i user_sees_no_improvement_in_f1", iterations_user_sees_no_improvment);

						if (iterations_user_sees_no_improvment > user_sees_no_improvment_const)
						{
							average_class_precision.push_back(f1);

							user_sees_no_improvement = true;
							ROS_INFO("\t\t[-]- user_sees_no_improvement");
							ROS_INFO("\t\t[-]- Finish"); 
							ROS_INFO("\t\t[-]- Number of taught categories= %i", number_of_taught_categories); 
							
							summary_of_experiment.open (evaluation_file.c_str(), std::ofstream::app);
							summary_of_experiment << "\n After " << user_sees_no_improvment_const <<" iterations, user sees no improvement in precision";
							summary_of_experiment.close();

							monitorPrecision( local_f1_vs_learned_category, f1);
							monitorF1VsLearnedCategory( f1_vs_learned_category, TP, FP, FN );

							
							ros::Duration duration = ros::Time::now() - begin_process;
							reportExperimentResult( average_class_precision,
													  number_of_instances, 
													  number_of_taught_categories,  
													  evaluation_file, duration);
							category_introduced.close();

							reportCurrentResults( TP, FP, FN, evaluation_file, true);
							
							reportAllExperimentalResults( TP, FP, FN, obj_num,			    
															average_class_precision,
															number_of_instances, 
															number_of_taught_categories,
															name_of_approach);
							
							plotSimulatedTeacherProgressInMatlab( experiment_number, protocol_threshold, precision_file);
							plotLocalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, protocol_threshold, local_f1_vs_learned_category.c_str());
							plotGlobalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, f1_vs_learned_category.c_str());
							plotNumberOfLearnedCategoriesVsIterationsInMatlab( experiment_number, category_introduced_txt.c_str());
							plotNumberOfStoredInstancesPerCategoryInMatlab( list_of_object_category);
							
							return 0 ;
						}
					}
				}
				else
				{
					float f1_system =0;
	
					if ((tp_tmp + fp_tmp)!=0)
					{
						precision = tp_tmp / double(tp_tmp+fp_tmp);
					}
					else
					{
						precision = 0;
					}		
					if ((tp_tmp+fn_tmp)!=0)
					{
						recall = tp_tmp / double(tp_tmp+fn_tmp);
					}
					else
					{
						recall = 0;
					}
					if ((precision + recall) != 0)
					{
						f1_system = 2 * (precision * recall) / (precision + recall);
					}
					else
					{
						f1_system = 0;
					}
					
					monitorPrecision( precision_file, f1_system);
				}
				k++; // k<-k+1 : number of classification result
				iterations	++;
			}//else	 
	    }//while
		monitorF1VsLearnedCategory (f1_vs_learned_category, TP, FP, FN );
	}
	
	ROS_INFO("\t\t[-]- Finish"); 
	ROS_INFO("\t\t[-]- Number of taught categories= %i", number_of_taught_categories); 

	//get toc
	ros::Duration duration = ros::Time::now() - begin_process;
	
	monitorPrecision( local_f1_vs_learned_category, f1);
	monitorF1VsLearnedCategory( f1_vs_learned_category, TP, FP, FN );

	reportCurrentResults( TP, FP, FN, evaluation_file, true);
	reportExperimentResult( average_class_precision,
							  number_of_instances, 
							  number_of_taught_categories,  
							  evaluation_file, duration);	
	
	category_introduced.close();
	reportAllExperimentalResults( TP, FP, FN, obj_num,			    
									average_class_precision,
									number_of_instances, 
									number_of_taught_categories,
									name_of_approach);
	
	plotSimulatedTeacherProgressInMatlab(experiment_number, protocol_threshold, precision_file);
	plotLocalF1VsNumberOfLearnedCategoriesInMatlab (experiment_number, protocol_threshold, local_f1_vs_learned_category.c_str());
	plotGlobalF1VsNumberOfLearnedCategoriesInMatlab (experiment_number, f1_vs_learned_category.c_str());
	plotNumberOfLearnedCategoriesVsIterationsInMatlab (experiment_number, category_introduced_txt.c_str());
	plotNumberOfStoredInstancesPerCategoryInMatlab( list_of_object_category);
		    
    return 0 ;
}

