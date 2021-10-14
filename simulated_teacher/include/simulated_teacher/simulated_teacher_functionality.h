// ############################################################################
//    
//   Created: 	2020
//   Author : 	
//   Email  :	
//   Purpose: 	This program follows the teaching protocol and autonomously
//		interact with the system using teach, ask and correct actions. 
// 		For each newly taught category, the average sucess of the system
// 		should be computed. To do that, the simulated teacher repeatedly 
// 		picks object views of the currently known categories from a 
// 		database and presents them to the system for checking whether 
// 		the system can recognize them. If not, the simulated teacher provides
// 		corrective feedback.
//   		
// 
// ############################################################################

#ifndef _SIMULATED_USER_LIB_H_
#define _SIMULATED_USER_LIB_H_

/* _________________________________
|                                 |
|           INCLUDES              |
|_________________________________| */

//Boost includes
#include <boost/make_shared.hpp>
//ROS includes
#include <ros/ros.h>

//PCL includes
#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>

//Eigen includes
#include <Eigen/Core>

//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>    
#include <string.h>
#include <cctype> // string manupolation
#include <math.h> 
#include <cstring>

//pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

//package includes
#include <backbone/spin_image.h>
#include <backbone/perception_msgs.h>
#include <backbone/print.h>
#include <backbone/CompleteRTOV.h>
#include <backbone/perception_db_serializer.h>
#include <simulated_teacher/simulated_teacher_functionality.h>


/* _________________________________
|                                 |
|           NAMESPACES            |
|_________________________________| */

using namespace pcl;
using namespace backbone;


/* _________________________________
|                                 |
|           Defines               |
|_________________________________| */

typedef PointXYZRGBA PointT; // define new type.


/* _________________________________
|                                 |
|        FUNCTION PROTOTYPES      |
|_________________________________| */

void set_home_address(string home_address);

/**
* @brief selectAnInstancefromSpecificCategory
* @param category_index (input parameter)
* @param instance_number (input/output parameter)
* @param Instance (input/output parameter)
* @return 
*/	
int selectAnInstancefromSpecificCategory( unsigned int category_index, 
                                          unsigned int &instance_number, 
                                          string &Instance);

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief generateSequence
* @param n (input parameter)
* @return 
*/	
vector <int> generateSequence (int n);

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief generateRrandomSequencesInstances
* @param path (input parameter)
* @return 
*/	
int generateRrandomSequencesInstances (string path);

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief generateRrandomSequencesCategories
* @param RunCount (input parameter)
* @return 
*/	
int generateRrandomSequencesCategories (int RunCount);

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief compute_precision_of_last_3n
* @param recognition_results (input parameter)
* @param number_of_taught_categories (input parameter)
* @return float precision 
*/	
float compute_precision_of_last_3n (vector <int> recognition_results, 
      int number_of_taught_categories);

////////////////////////////////////////////////////////////////////////////////////////////
float computeF1OfLast3n (vector <int> recognition_results ,
          int number_of_taught_categories);

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief report_current_results
* @param TP (input parameter)
* @param FP (input parameter)
* @param FN (input parameter)
* @param fname (input parameter)
* @param global (input parameter) 
* @return 
*/	
int compute_Precision_Recall_Fmeasure_of_last_3n (vector <int> recognition_results ,
                                                  int number_of_taught_categories, 
                                                  float &Precision, float &Recall, float &F1);

////////////////////////////////////////////////////////////////////////////////////////////
void monitorPrecision (string precision_file, float Precision );

////////////////////////////////////////////////////////////////////////////////////////////
void monitorF1VsLearnedCategory (string f1_vs_learned_caegory, int TP, int FP, int FN );

////////////////////////////////////////////////////////////////////////////////////////////
void reportCurrentResults(int TP, int FP, 
    int FN, string fname, 
    bool global);

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief report_category_introduced
* @param fname (input parameter)
* @param cat_name (input parameter) 
* @return 
*/	
void report_category_introduced(string fname, string cat_name);

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief report_category_introduced
* @param fname (input parameter)
* @param Precision (input parameter) 
* @return 
*/	
void report_precision_of_last_3n(string fname, double Precision);

////////////////////////////////////////////////////////////////////////////////////////////
void reportF1OfLast3n(string fname, double F1);

////////////////////////////////////////////////////////////////////////////////////////////
int reportExperimentResult (vector <float> average_class_precision,
        int number_of_stored_instances, 
        int number_of_taught_categories,  
        string fname, ros::Duration duration);

////////////////////////////////////////////////////////////////////////////////////////////
string fixedLength (string name , size_t length);
////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief extractObjectName
* @param Object_name_orginal (input parameter)
* @return string (output parameter)
*/	
string extractObjectNameSimulatedUser (string object_name_orginal );

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief extractCategoryName
* @param InstancePath (input parameter)
* @return string (output parameter)
*/	
string extractCategoryName (string InstancePath);

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief addObjectViewHistogramInSpecificCategory
* @param cat_name (input parameter)
* @param cat_id (input parameter)
* @param track_id (input parameter)
* @param view_id (input parameter)
* @param objectViewHistogram (input parameter) 
* @param pp (input parameter) 
* @return 
*/	
int addObjectViewHistogramInSpecificCategory(std::string cat_name, 
        unsigned int cat_id, 
        unsigned int track_id,
        unsigned int view_id,
        SITOV objectViewHistogram,
        PrettyPrint &pp );

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief objectRepresentationBagOfWords
* @param cluster_center (input parameter)
* @param object_spin_images (input parameter)
* @param object_representation (input/output parameter)
* @return 
*/	
int objectRepresentationBagOfWords (vector <SITOV> cluster_center, 
      vector <SITOV> object_spin_images, 
      SITOV  &object_representation );

////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief readClusterCenterFromFile
* @param path (input parameter)
* @return 
*/	
vector <SITOV>  readClusterCenterFromFile (string path);

////////////////////////////////////////////////////////////////////////////////////////////
void plotSimulatedTeacherProgressInMatlab( int RunCount, float P_Threshold, string precision_file);
void plotLocalF1VsNumberOfLearnedCategoriesInMatlab( int RunCount, float P_Threshold, string local_F1_vs_learned_category);
void plotGlobalF1VsNumberOfLearnedCategoriesInMatlab( int RunCount, string global_F1_vs_learned_category);
void plotNumberOfLearnedCategoriesVsIterationsInMatlab( int RunCount, string Number_of_learned_categories_vs_Iterations);
void plotNumberOfStoredInstancesPerCategoryInMatlab( vector <ObjectCategory> list_of_object_category);

////////////////////////////////////////////////////////////////////////////////////////////
bool fexists(std::string filename);

////////////////////////////////////////////////////////////////////////////////////////////
int reportAllExperimentalResults( int TP, int FP, int FN,int obj_num,			    
                                  vector <float> average_class_precision,
                                  float number_of_stored_instances, 
                                  int number_of_taught_categories,
                                  string name_of_approach );

////////////////////////////////////////////////////////////////////////////////////////////
int sum_all_experiments_results(  int iterations,
                                  float Success_Precision		,			    
                                  float average_class_precision_value,
                                  float number_of_stored_instances, 
                                  int number_of_taught_categories,
                                  string name_of_approach );

////////////////////////////////////////////////////////////////////////////////////////////

int average_all_experiments_results( int total_number_of_experiments,
                                     string name_of_approach);

/////////////////////////////////////////////////////////////////////////////////////////////////////

int introduceNewInstanceLocalHDP(   string home_address,
                                    std::string object_path, 
                                    unsigned int cat_id, 
                                    unsigned int track_id, 
                                    unsigned int view_id,
                                    int spin_image_width_int,
                                    float spin_image_support_lenght_float,
                                    float voxel_size_for_keypoints,
                                    vector <SITOV> dictionary,
                                    ros::ServiceClient topic_modelling_server,
                                    PrettyPrint &pp );

/////////////////////////////////////////////////////////////////////////////////////////////////////
int introduceNewCategoryLocalHDP(	string home_address, 
                                    int class_index,
                                    unsigned int &track_id,
                                    unsigned int &instance_number,
                                    string fname,
                                    int spin_image_width_int,
                                    float spin_image_support_lenght_float,
                                    float voxel_size_for_keypoints,
                                    vector <SITOV> dictionary,
                                    ros::ServiceClient topic_modelling_server,
                                    PrettyPrint &pp);

#endif

