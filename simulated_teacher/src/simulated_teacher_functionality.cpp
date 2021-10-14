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

/* _________________________________
|                                 |
|           INCLUDES              |
|_________________________________| */

//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>

//ROS & PCL includes
#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/package.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

//perception db includes
#include <backbone/perception_msgs.h>
#include <backbone/perception_db.h>
#include <backbone/perception_db_serializer.h>
#include <backbone/spin_image.h>
#include <backbone/print.h>

// define topic modelling service here
#include <topic_modeling_services/topic_modelling.h>


#define recognitionThershold 20000

using namespace pcl;
using namespace std;
using namespace ros;
  
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBA T; 

/* _______________________________
|                                 |
|         Global variable         |
|_________________________________| */

PerceptionDB* _pdb; //initialize the class
int track_id_gloabal =1;
ofstream allFeatures;
std::string PCDFileAddressTmp;
int Hist_lenght = 0;
std::string home_directory_address= "";

/* _____________________
|                       |
|        Functions      |
|_______________________| */

/////////////////////////////////////////////////////////////////////////////////////////////////////
void set_home_address(string home_address)
{
	home_directory_address = home_address;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
string fixedLength (string name , size_t length)
{
	while (name.length() < length)
    { 
		name += " ";
    }
    return (name);
} 

/////////////////////////////////////////////////////////////////////////////////////////////////////
string extractObjectNameSimulatedUser (string object_name_orginal )
{
    std:: string object_name;
    std:: string tmp_object_name = object_name_orginal;// for writing object name in result file;
    int rfind = tmp_object_name.rfind("//")+2;       
    int len = tmp_object_name.length();

    object_name = object_name_orginal.substr(rfind, tmp_object_name.length());

	while (object_name.length() < 30)
    { 
		object_name += " ";
    }

    return (object_name);
} 

/////////////////////////////////////////////////////////////////////////////////////////////////////
string extractCategoryName (string InstancePath )
{
    string categoryName="";	    
    int ffind = InstancePath.find("//")+2;  
    int lfind =  InstancePath.find("_Cat");       

    for (int i=0; i<(lfind-ffind); i++)
    {
		categoryName += InstancePath.at(i+ffind);
    }
    return (categoryName);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
int addObjectViewHistogramInSpecificCategory(std::string cat_name,
					      unsigned int cat_id, 
					      unsigned int track_id,
					      unsigned int view_id, 
					      SITOV objectViewHistogram,
					      PrettyPrint &pp )
{
    SITOV msg_in;
    RTOV _rtov;
    _rtov.track_id = track_id;
    _rtov.view_id = view_id;

	msg_in = objectViewHistogram;
	msg_in.spin_img_id = 1;

	uint32_t sp_size = ros::serialization::serializationLength(msg_in);

	boost::shared_array<uint8_t> sp_buffer(new uint8_t[sp_size]);
	PerceptionDBSerializer<boost::shared_array<uint8_t>, SITOV>::serialize(sp_buffer, msg_in, sp_size);
	leveldb::Slice sp_s((char*)sp_buffer.get(), sp_size);
	std::string sp_key = _pdb->makeSIKey(key::SI, track_id, view_id, 1 );

	//Put slice to the db
	_pdb->put(sp_key, sp_s); 

	//create a list of key of spinimage
	_rtov.sitov_keys.push_back(sp_key);

    uint32_t v_size = ros::serialization::serializationLength(_rtov);

    boost::shared_array<uint8_t> v_buffer(new uint8_t[v_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::serialize(v_buffer, _rtov, v_size);	

    leveldb::Slice v_s((char*)v_buffer.get(), v_size);

    std::string v_key = _pdb->makeKey(key::RV, track_id, view_id);
    ROS_INFO("\t\t [-] v_key: %s, view_id: %i, track_id: %i", v_key.c_str(), view_id, track_id);

    //Put one view to the db
    _pdb->put(v_key, v_s);

    ObjectCategory _oc;

    std::string oc_key = _pdb->makeOCKey(key::OC, cat_name, cat_id);

    std::string str_oc;
    _pdb->get(oc_key, &str_oc);
    uint32_t oc_size = str_oc.length();
    if (oc_size != 0) //Object category exist.
    {
        boost::shared_array<uint8_t> oc_dbuffer(new uint8_t[oc_size]);
        memcpy(oc_dbuffer.get(), str_oc.data(), str_oc.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

        //deserialize Msg 
        backbone::PerceptionDBSerializer<boost::shared_array<uint8_t>, backbone::ObjectCategory>::deserialize(oc_dbuffer, _oc, oc_size);
    }

    _oc.cat_name = cat_name;
    _oc.cat_id = cat_id ;
    _oc.rtov_keys.push_back(v_key);

	_oc.icd = 0.00001;

    oc_size = ros::serialization::serializationLength(_oc);

    pp.info(std::ostringstream().flush() << _oc.cat_name.c_str() << " category has " << _oc.rtov_keys.size() << " objects.");

    boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::serialize(oc_buffer, _oc, oc_size);	
    leveldb::Slice ocs((char*)oc_buffer.get(), oc_size);
    _pdb->put(oc_key, ocs);

    return (1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int differenceBetweenSpinImage( SITOV  sp1, 
								SITOV  sp2,
								float &difference)	
{
	difference = 0;
	if (sp1.spin_image.size() != sp2.spin_image.size())
	{
		return 0;
	}

	for (size_t j = 0; j < sp1.spin_image.size(); j++)
	{
		difference += pow( sp1.spin_image.at(j) - sp2.spin_image.at(j), 2.0);
	}

	return 1;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int objectRepresentationBagOfWords (vector <SITOV> cluster_center, 
				    vector <SITOV> object_spin_images, 
				    SITOV  &object_representation )
{
   
    if (object_spin_images.size() == 0 )
    {
		ROS_ERROR("Error: size of object view spin images is zero- could not represent based on BOW");
    }

    for (size_t i = 0; i < cluster_center.size(); i++)
    {
		object_representation.spin_image.push_back(0);
    }
    
    //ROS_INFO("\t\t[-]- size of object view histogram = %ld", object_representation.spin_image.size());
    
    for (size_t i = 0; i < object_spin_images.size(); i++)
    {		
		SITOV sp1;
		sp1=object_spin_images.at(i);
		
		float diffrence=100;
		float diff_temp = 100;
		int id=0;

		for (size_t j = 0; j < cluster_center.size(); j++)
		{
			SITOV sp2;
			sp2= cluster_center.at(j);
			if (!differenceBetweenSpinImage(sp1, sp2, diffrence))
			{	
				ROS_INFO("\t\t[-]- size of spinimage of cluster center= %ld", sp2.spin_image.size());
				ROS_INFO("\t\t[-]- size of spinimage of the object= %ld" ,sp1.spin_image.size());
				ROS_ERROR("Error comparing spin images");
				return 0;	
			}
			//ROS_INFO("\t\t[-]- diffrence[%ld,%ld] = %f    diff_temp =%f",i ,j, diffrence, diff_temp);
			if ( diffrence < diff_temp)
			{
				diff_temp = diffrence;
				id = j;
			} 
		}		
		object_representation.spin_image.at(id)++;
    }
       
    return (1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
vector <SITOV>  readClusterCenterFromFile (string path)
{
    std::ifstream clusterCenter(path.c_str());
    std::string clusterTmp;
    vector <SITOV> clusterCenterSpinimages;
    std::setprecision(15);
    while (clusterCenter.good ())// read cluster Center
    { 	
		std::getline (clusterCenter, clusterTmp);
		if(clusterTmp.empty () || clusterTmp.at (0) == '#') // Skip blank lines or comments
			continue;
		
		string element = "";
		SITOV sp;
		size_t j =0;
		size_t k =0;
		
		//ROS_INFO("\t\t[-]- size = %ld ", clusterTmp.length());
		float value;
		for (size_t i = 0; i < clusterTmp.length(); i++)
		{
			char ch = clusterTmp[i];
			//ROS_INFO("\t\t[-]- ch = %c", ch);
			if (ch != ',')
			{
				element += ch;
			}
			else
			{
				//ROS_INFO("\t\t[-]- element[%ld,%ld] = %s",k ,j, element.c_str());
				value = atof(element.c_str());
				//ROS_INFO("\t\t[-]- element[%ld,%ld] = %f",k ,j, value);
				sp.spin_image.push_back(value);   
				element = "";    
				j++;
			} 
		}
		value = atof(element.c_str());
		sp.spin_image.push_back(value);
		clusterCenterSpinimages.push_back(sp);
		k++;
    }
    
   return (clusterCenterSpinimages);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
int selectAnInstancefromSpecificCategory(unsigned int category_index, 
					 unsigned int &instance_number, 
					 string &Instance)
{
    std::string path;
    path =  home_directory_address +"/Category/Category.txt";
    ROS_INFO("path = %s",path.c_str());
    ROS_INFO("\t\t[-] category index = %d",category_index);
    ROS_INFO("\t\t[-] instance_number = %d",instance_number);

    std::ifstream listOfObjectCategoriesAddress (path.c_str());
    std::string categoryAddresstmp;
    std::string categoryName;

    unsigned int cat_index = 0;
    while ((listOfObjectCategoriesAddress.good()) && (cat_index < category_index))
    {
		std::getline (listOfObjectCategoriesAddress, categoryAddresstmp);
		if(categoryAddresstmp.empty () || categoryAddresstmp.at (0) == '#') // Skip blank lines or comments
			continue;

		if (cat_index < category_index)
			cat_index++;
    }
    
    if (cat_index != category_index)
    {
		ROS_INFO("\t\t[-]-The file doesn't exist - condition : cat_index != category_index, cat_index = %i", cat_index);
		return -1;
    }
    
    path = home_directory_address +"/"+ categoryAddresstmp.c_str();
    
    std::ifstream categoyInstances (path.c_str());
    std::string PCDFileAddressTmp;
    
    unsigned int inst_number =0;
    while ((categoyInstances.good ()) && (inst_number < instance_number))// read instances of a category 
    {	
		std::getline (categoyInstances, PCDFileAddressTmp);
		if(PCDFileAddressTmp.empty () || PCDFileAddressTmp.at (0) == '#') // Skip blank lines or comments
			continue;
		if (inst_number < instance_number)
	    inst_number ++;
    }
    if (inst_number < instance_number)
    {
		ROS_INFO("\t\t[-]- category path= %s", path);
		ROS_INFO("\t\t[-]-The file doesn't exist - condition : inst_number < instance_number -- inst_number = %i", inst_number);
		return -1;
    }
    
    Instance=PCDFileAddressTmp;
    instance_number++;
    return 0;
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
vector <int> generateSequence (int n)
{
    /* initialize random seed: */
    srand (time(NULL));
    vector <int> sequence;
    while( sequence.size() < n )
    {    
		/* generate random number between 1 and 10: */
		int num = rand() % n + 1;
		bool flag = false;
		for (int j= 0; j < sequence.size(); j++)
		{
			if (num == sequence.at(j))
			{
				flag= true;
				break;
			}
		}
		if (flag == false)
		{
			sequence.push_back(num);
		}
    }
    return(sequence);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesInstances (string path)
{    
    string path1= home_directory_address + path;    
    std::ifstream listOfObjectInstancesAddress (path1.c_str());
    string InstanceAddresstmp = "";
    unsigned int number_of_exist_instances = 0;
    while (listOfObjectInstancesAddress.good()) 
    {
		std::getline (listOfObjectInstancesAddress, InstanceAddresstmp);
		if(InstanceAddresstmp.empty () || InstanceAddresstmp.at (0) == '#') // Skip blank lines or comments
			continue;
		number_of_exist_instances++;
    }
    
    InstanceAddresstmp = "";
    vector <int> instances_sequence = generateSequence (number_of_exist_instances);
    std::ofstream instances;
    string path2 = home_directory_address + path;
    path2.resize(path2.size()-12);
    path2+= ".txt";
    //ROS_INFO("\n reorder category = %s \n",path2.c_str());

    instances.open (path2.c_str(), std::ofstream::out);
    for (int i =0; i < instances_sequence.size(); i++)
    {
		std::ifstream listOfObjectCategories (path1.c_str());
		int j = 0;
		while ((listOfObjectCategories.good()) && (j < instances_sequence.at(i)))
		{
			std::getline (listOfObjectCategories, InstanceAddresstmp);
			j++;
		}
		instances << InstanceAddresstmp.c_str()<<"\n";
		//ROS_INFO("\n instance= %s \n",InstanceAddresstmp.c_str());
    }
    instances.close();
    
    return (0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesCategories (int RunCount)
{

    std::string path;
    ROS_INFO ("home_directory_address = %s", home_directory_address.c_str());

    path = home_directory_address +"/Category/Category_orginal.txt";
    
    std::ifstream listOfObjectCategoriesAddress (path.c_str());
    ROS_INFO ("path = %s", path.c_str());

    string categoryAddresstmp = "";
    unsigned int number_of_exist_categories = 0;
    while (listOfObjectCategoriesAddress.good()) 
    {
		std::getline (listOfObjectCategoriesAddress, categoryAddresstmp);
		if(categoryAddresstmp.empty () || categoryAddresstmp.at (0) == '#') // Skip blank lines or comments
			continue;
		number_of_exist_categories++;
    }                	
    ROS_INFO ("number of category = %i", number_of_exist_categories);

    vector <int> categories_sequence = generateSequence (number_of_exist_categories);
    std::ofstream categoies;
    string path2 = home_directory_address +"/Category/Category.txt";
    categoies.open (path2.c_str(), std::ofstream::out);
    for (int i =0; i < categories_sequence.size(); i++)
    {
		std::ifstream listOfObjectCategories (path.c_str());
		int j = 0;
		while ((listOfObjectCategories.good()) && (j < categories_sequence.at(i)))
		{
			std::getline (listOfObjectCategories, categoryAddresstmp);
			generateRrandomSequencesInstances(categoryAddresstmp.c_str());
			j++;
		}
		categoryAddresstmp.resize(categoryAddresstmp.size()-12);
		categoryAddresstmp+= ".txt";
		categoies << categoryAddresstmp.c_str()<<"\n";
    }

    return 0 ;  
}

// /////////////////////////////////////////////////////////////////////////////////////////////////////
float computeF1OfLast3n (vector <int> recognition_results ,
				     int number_of_taught_categories)
{
    ROS_INFO("\t\t[-]- ^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&");
    ROS_INFO("\t\t[-]- Inside compute_precision_of_last_3n function");
    ROS_INFO("\t\t[-]- size of recognition result = %d",recognition_results.size() );
    if (recognition_results.size() < 1)
    {
		printf(" Error: size of recognition result array is %i",recognition_results.size() );
		return 0;
    }
    for (int i =0; i< recognition_results.size(); i++)
    {
	    printf("%d, ",recognition_results.at(i) );
    }
    printf("\n");
    
    float precision = 0;
    int TP = 0; int FP = 0; int FN = 0;
    
    int win_size = 3*number_of_taught_categories;
    ROS_INFO("\ni start from %d till %d",recognition_results.size() - win_size -1, recognition_results.size()-1 );
    ROS_INFO("windows size = %d",win_size );
    ROS_INFO("number_of_taught_categories = %d",number_of_taught_categories );

    
    for (int i = recognition_results.size()-1; i > recognition_results.size() - win_size ; i--)
    {
		int result = recognition_results.at(i);
		if (result==1)
		{
			TP++;
		}
		else if (result==2)
		{
			FP++;
		}
		else if (result==3)
		{
			FN++;
		}
		else if (result==4)
		{
			FP++;FN++;
		}
    }
    
    ROS_INFO("\t\t[-]- number of TP= %d, FP= %d, FN=%d", TP,FP,FN); 
    
    float Precision = TP/double (TP+FP);
    float Recall = TP/double (TP+FN);    
    float f1 = 2 * (Precision * Recall )/(Precision + Recall );
    ROS_INFO("\t\t[-]- F1 = %f", f1); 
        
    return(f1);
}

///////////////////////////////////////////////////////////////////////////////////////
void monitorPrecision (string precision_file, float Precision )
{
    std::ofstream PrecisionMonitor;
    PrecisionMonitor.open (precision_file.c_str(), std::ofstream::app);
    PrecisionMonitor.precision(4);
    PrecisionMonitor << Precision<<"\n";
    PrecisionMonitor.close();    
}

///////////////////////////////////////////////////////////////////////////////////////
void monitorF1VsLearnedCategory (string f1_learned_caegory, int TP, int FP, int FN )
{
    double Precision, Recall, F1;
    if ((TP+FP)!=0)
    {
		Precision = TP/double (TP+FP);
    }
    else
    {
		Precision = 0;
    }		
    if ((TP+FN)!=0)
    {
		Recall = TP/double (TP+FN);
    }
    else
    {
		Recall = 0;
    }
    if ((Precision + Recall)!=0)
    {
	  	F1 = 2 * (Precision * Recall )/(Precision + Recall );
    }
    else
    {
		F1 = 0;
    }  
  
    std::ofstream f1_vs_learned_caegory;
    f1_vs_learned_caegory.open (f1_learned_caegory.c_str(), std::ofstream::app);
    f1_vs_learned_caegory.precision(4);
    f1_vs_learned_caegory << F1<<"\n";
    f1_vs_learned_caegory.close();    
}

///////////////////////////////////////////////////////////////////////////////////////
void reportCurrentResults(int TP, int FP, int FN, string fname, bool global)
{
	double Precision = TP/double (TP+FP);
	double Recall = TP/double (TP+FN);
	
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file.precision(4);
	if (global)
		Result_file << "\n\n\t******************* Global *********************";
	else	Result_file << "\n\n\t******************* Lastest run ****************";
	Result_file << "\n\t\t - True  Positive = "<< TP;
	Result_file << "\n\t\t - False Positive = "<< FP;
	Result_file << "\n\t\t - False Negative = "<< FN;
	Result_file << "\n\t\t - Precision  = "<< Precision;
	Result_file << "\n\t\t - Recall = "<< Recall;
	Result_file << "\n\t\t - F1 = "<< 2 * (Precision * Recall )/(Precision + Recall );
	Result_file << "\n\n\t************************************************\n\n";
		
	Result_file << "\n------------------------------------------------------------------------------------------------------------------------------------";
	Result_file.close();
}

///////////////////////////////////////////////////////////////////////////////////////
void report_category_introduced(string fname, string cat_name)
{
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file << "\n\t********************************************";
	Result_file << "\n\t\t - "<< cat_name.c_str() <<" Category Introduced";
	Result_file << "\n\t********************************************";
	Result_file.close();
}

///////////////////////////////////////////////////////////////////////////////////////
void report_precision_of_last_3n(string fname, double Precision)
{
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file << "\n\t*********** Precision of last 3n **************";
	Result_file << "\n\t\t - precision = "<< Precision;
	Result_file << "\n\t********************************************";
	Result_file.close();
}

///////////////////////////////////////////////////////////////////////////////////////
void reportF1OfLast3n(string fname, double F1)
{
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file << "\n\t*********** F1 of last 3n **************";
	Result_file << "\n\t\t - F1 = "<< F1;
	Result_file << "\n\t********************************************";
	Result_file.close();
}

///////////////////////////////////////////////////////////////////////////////////////
int reportExperimentResult (vector <float> average_class_precision,
			     int number_of_stored_instances, 
			     int number_of_taught_categories,  
			     string fname, ros::Duration duration)
{
   
    float average_class_precision_value =0;
    for (int i =0; i<average_class_precision.size(); i++)
    {
		average_class_precision_value+=average_class_precision.at(i); 	    
    }
    average_class_precision_value=average_class_precision_value/average_class_precision.size();
    double duration_sec = duration.toSec();
    
    std::ofstream Result;
    Result.open (fname.c_str(), std::ofstream::app);
    Result.precision(4);
    Result << "\n\n\t************** Expriment Result ****************";
    Result << "\n\t\t - Average_class_precision = "<< average_class_precision_value;
    Result << "\n\t\t - All stored instances = "<< number_of_stored_instances;
    Result << "\n\t\t - Number of taught categories = "<< number_of_taught_categories;
    Result << "\n\t\t - Average number of instances per category = "<< float (number_of_stored_instances) / float (number_of_taught_categories);	
    Result << "\n\t\t - This expriment took " << duration_sec << " secs";
    Result << "\n\n\t************************************************\n\n";
    Result.close();
    return (0);
}

///////////////////////////////////////////////////////////////////////////////////////
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
									PrettyPrint &pp )
{

    string category_name = extractCategoryName(object_path);
    
    if(object_path.empty () || object_path.at (0) == '#' || category_name == "Category//Unk") // Skip blank lines or comments
    {
		return 0;
    }
    
    object_path = home_address +"/"+ object_path.c_str();

    //load a PCD object  
    boost::shared_ptr<PointCloud<PointT> > PCD_file (new PointCloud<PointT>);
    if (io::loadPCDFile <PointXYZRGBA> (object_path.c_str(), *PCD_file) == -1)
    {	
	    ROS_ERROR("\t\t[-]-Could not read given object %s :", object_path.c_str());
	    return(0);
    }
    else
    {
	    ROS_INFO("\t\t[1]-Loaded a point cloud: %s", object_path.c_str());
    }
    
	/* ___________________________________________________
	|                                                     |
	|  Compute the Spin-Images for the given test object  |
	|_____________________________________________________| */
	
	boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
	pcl::VoxelGrid<PointT > voxelized_point_cloud;	
	voxelized_point_cloud.setInputCloud (PCD_file);
	voxelized_point_cloud.setLeafSize (0.005, 0.005, 0.005);
	voxelized_point_cloud.filter (*target_pc);
	ROS_INFO( "The size of converted point cloud  = %d ", target_pc->points.size() );
	
	//Declare a boost share ptr to the spin image msg		  
	boost::shared_ptr< vector <SITOV> > objectViewSpinImages;
	objectViewSpinImages = (boost::shared_ptr< vector <SITOV> >) new (vector <SITOV>);
	
	boost::shared_ptr<PointCloud<PointT> > uniform_keypoints (new PointCloud<PointT>);
	boost::shared_ptr<pcl::PointCloud<int> >uniform_sampling_indices (new PointCloud<int>);
	keypointSelection(   target_pc, 
						voxel_size_for_keypoints,
						uniform_keypoints,
						uniform_sampling_indices);

	ROS_INFO ("number of keypoints = %i", uniform_keypoints->points.size());
	
	
	if (!estimateSpinImages(target_pc, 
				0.01 /*downsampling_voxel_size*/, 
				0.05 /*normal_estimation_radius*/,
				spin_image_width_int /*spin_image_width*/,
				0.0 /*spin_image_cos_angle*/,
				1 /*spin_image_minimum_neighbor_density*/,
				spin_image_support_lenght_float /*spin_image_support_lenght*/,
				objectViewSpinImages,
				uniform_sampling_indices /*subsample spinimages*/))
	{
		ROS_INFO("Could not compute spin images");
		return (0);
	}
	
	// represent object based on BOW
	SITOV object_representation;
	objectRepresentationBagOfWords (dictionary, *objectViewSpinImages, object_representation);


	SITOV local_HDP_object_representation;
	/// call topic_modelling (local HDP) service to represent the given object
	topic_modeling_services::topic_modelling srv;
	srv.request.bag_of_words_representation = object_representation.spin_image;
	srv.request.ground_truth_label = category_name;
	srv.request.teach_data = true;	

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
	}
	ROS_INFO("\t\t[-] Size of local_HDP_object_representation is %d", local_HDP_object_representation.spin_image.size());
	addObjectViewHistogramInSpecificCategory(category_name, 1, track_id, 1, local_HDP_object_representation , pp);
    ROS_INFO("\t\t[-]-%s created...",category_name.c_str());
        
    return (0);
}

///////////////////////////////////////////////////////////////////////////////////////
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
									PrettyPrint &pp)
				  
{
	string instance_path;
	for(int i = 0; i < 3 ; i++)  // 2 instances would be enough
	{
	    ROS_INFO("\t\t[-]-inside introduceNewCategoryHistogram function Instance number %d", instance_number);
	    if (selectAnInstancefromSpecificCategory(class_index, instance_number, instance_path)==-1)
	    {	
			ROS_ERROR("\t\t[-]-The object view or category does not exist");
			return -1;// ERROR : file doesn't exist
	    }
	    
	    int cat_id = 1; // cat_id is always 1 beacuse we use cat_name key:<cat_name><cat_id>
	    int view_id = 1; // view_id is always 1 beacuse we use TID key:<TID><VID>
	    introduceNewInstanceLocalHDP (  home_address,
										instance_path, 
										cat_id, track_id, view_id,
										spin_image_width_int,
										spin_image_support_lenght_float,
										voxel_size_for_keypoints,
										dictionary,
				   						topic_modelling_server,
										pp );


	    track_id++;
	    //view_id ++; // in this implementation we consider VID as a constant
	}

	// extracting the category name 
	string categoryName=extractCategoryName(instance_path);
	ROS_INFO("\n extractCategoryName %s", categoryName.c_str()); 
	report_category_introduced(fname,categoryName.c_str());
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////

void plotSimulatedTeacherProgressInMatlab( int RunCount, float P_Threshold, string precision_file)
{
 
    // If the text inside the plot is not well-appear, please install the font in ubuntu using the following commands.
    // sudo apt-get install xfonts-75dpi
    // sudo apt-get install xfonts-75dpi
    
    char run_count [10];
	vector<float> acc;

    sprintf( run_count, "%d",RunCount );
    string path = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/graph1.m";
        
    ROS_INFO("\t\tpath = %s", path.c_str());

    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "close all;\nfigure;\nhold on;\ngrid on;";
    matlabFile << "\nset(gca,'LineStyleOrder', '-');";
    matlabFile << "\nPrecision= [";
    
    std::string value;
    std::ifstream ReadPrecisionMonitor (precision_file.c_str());
    std::getline (ReadPrecisionMonitor, value);
    matlabFile << value;
	acc.push_back(atof (value.c_str()));

    int iteration = 1;
    while (ReadPrecisionMonitor.good())
    {
		std::getline (ReadPrecisionMonitor, value);
		matlabFile << ", "<< value;
		acc.push_back(atof (value.c_str()));
		iteration++;
    }
    
    int remain = iteration % 10;
    matlabFile << "];\naxis([0,"<< iteration + 10 - remain <<",0,1.2]);\nplot (Precision, 'LineWidth',2);";
    matlabFile << "\nxlabel('Question / Correction Iterations','FontSize',15);\nylabel('Protocol Accuracy','FontSize',15);";

    matlabFile << "\nset(gca,'LineStyleOrder', '--');";
    //draw a threshold line
    matlabFile << "\nline([0 "<< iteration + 10 - remain <<"],["<< P_Threshold << " " <<P_Threshold <<"] ,'Color',[0 0 0 0.3], 'LineWidth',1);";
    matlabFile << "\ntext(" << iteration - remain - 10 << "," << P_Threshold + 0.05 <<",'Thereshold', 'FontSize',12, 'Interpreter','Latex');";

    iteration = 0;
    string path_tmp = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/Category_Introduced.txt";
    std::ifstream read_category_introduced (path_tmp.c_str());

    string path_tmp2 = home_directory_address+ "/Category/Category.txt";
    std::ifstream category_name (path_tmp2.c_str());
    string cat_name ="";
    
    value = "";
    string value_tmp;
    // draw a line 
    std::getline (read_category_introduced, value_tmp);
    matlabFile << "\nline(["<< iteration<< ","<< iteration <<"] ,[0.05,1],'Color',[1 0 0], 'LineWidth',1);";
    
    // add category name to the graph
    std::getline (category_name, cat_name);
    cat_name = extractCategoryName(cat_name);
    
    int lfind =  cat_name.find("_");
    if (lfind > 0) 
		cat_name.replace(lfind, 1,1, '-');


    float text_Y_pos = 0.05;
    bool flg=true;
    matlabFile << "\ntext("<<iteration<<".5, 0.05 ,'"<< cat_name <<"' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex' ,'BackgroundColor', [1,0,0,0.2]);";
        
    std::getline (read_category_introduced, value_tmp);
    matlabFile << "\nline(["<< iteration<< ","<< iteration <<"] ,[0.1,1],'Color',[1 0 0], 'LineWidth',1);";	    
    std::getline (category_name, cat_name);
    
    cat_name = extractCategoryName(cat_name);
    lfind =  cat_name.find("_");
    if (lfind > 0) 
	cat_name.replace(lfind, 1,1, '-');
    
    matlabFile << "\ntext("<<iteration<<".5, 0.1,'"<< cat_name <<"' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex', 'BackgroundColor', [1,0,0,0.2] );";

    iteration = 1;
    float y = 0.15;
    while (read_category_introduced.good())
    {

		std::getline (read_category_introduced, value);
		if (strcmp(value.c_str(),"1") == 0)
		{
			std::getline (category_name, cat_name);

			cat_name = extractCategoryName(cat_name);
			lfind = cat_name.find("_");
			if (lfind > 0) 
			{
				cat_name.replace(lfind, 1,1, '-');
			}

			if (acc.at(iteration-1) == 0)
			{
				matlabFile << "\nline([" << iteration << "," << iteration <<"] ,["<< 1 - y <<", 0],'Color',[1 0 0], 'LineWidth',1);";	    
				matlabFile << "\ntext(" << iteration <<".5," << 1 - y <<" ,'"<< cat_name <<"' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex', 'BackgroundColor', [1,0,0,0.2]);";
			}
			else
			{
				matlabFile << "\nline(["<< iteration<< ","<< iteration <<"] ,["<< y <<", 1],'Color',[1 0 0], 'LineWidth',1);";	    
				matlabFile << "\ntext("<<iteration<<".5,"<< y <<" ,'"<< cat_name <<"' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex', 'BackgroundColor', [1,0,0,0.2]);";
			}
			if (iteration == 1) 
				iteration ++ ;
			


			if (y <=0.55)
			{
				y+=0.05;
			}
			else
			{
				y=0.05;
			}
		}
		else
		{
			iteration++;
		}		
    }
    matlabFile.close();
    ROS_INFO("\t\tMATLAB file created...");
}

/////////////////////////////////////////////////////////////////////////////////////////////
void plotLocalF1VsNumberOfLearnedCategoriesInMatlab( int RunCount, 
													 float P_Threshold, 
													 string local_F1_vs_learned_category)
{
    // If the text inside the plot is not well-appear, please install the font in ubuntu using the following commands.
    // sudo apt-get install xfonts-75dpi
    
    char run_count [10];
    sprintf( run_count, "%d",RunCount );
    string path = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/local_F1_vs_learned_category.m";
        
    ROS_INFO("\t\tpath = %s", path.c_str());

    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "close all;\nfigure ();\nhold on;\ngrid on;";
    matlabFile << "\nset(gca,'LineStyleOrder', '--');";
    matlabFile << "\ntext(1,0.685 ,'Threshold' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex');";

    matlabFile << "\nlocalF1= [";
    
    std::string value;
    std::ifstream Readlocal_F1 (local_F1_vs_learned_category.c_str());
    std::getline (Readlocal_F1, value);
    matlabFile << value;

    int iteration = 1;
    while (Readlocal_F1.good())
    {
		std::getline (Readlocal_F1, value);
		if(value.empty ()) // Skip blank lines or comments
			continue;
			
		matlabFile <<", " << value ;
		iteration++;
    }
    
    int remain = iteration % 5;
    matlabFile << "];\nline([0 "<< iteration + 5 - remain <<"],["<< P_Threshold << " " <<P_Threshold <<"] ,'Color',[0 0 0], 'LineWidth',1);";
    matlabFile << "\naxis([0,"<< iteration + 5 - remain <<",0.5,1.05]);"; 
    matlabFile << "\nplot(1:size(localF1,2), localF1(1:size(localF1,2)),'-.O', 'Color',[1 0 1], 'LineWidth',2.);";
    matlabFile << "\nxlabel('Number of Learned Categories','FontSize',15);";
    matlabFile << "\nylabel('Protocol Accuracy','FontSize',15);";

    matlabFile.close();
    ROS_INFO("\t\tMATLAB file created...");
}

/////////////////////////////////////////////////////////////////////////////////////////////
void plotGlobalF1VsNumberOfLearnedCategoriesInMatlab(int RunCount, 
								    				  string global_F1_vs_learned_category)
{
    // If the text inside the plot is not well-appear, please install the font in ubuntu using the following commands.
    // sudo apt-get install xfonts-75dpi
    // sudo apt-get install xfonts-75dpi
    
    char run_count [10];
    sprintf( run_count, "%d",RunCount );
    string path = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/global_F1_vs_learned_category.m";
        
    ROS_INFO("\t\tpath = %s", path.c_str());

    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "close all;\nfigure ();\nhold on;\ngrid on;";
    matlabFile << "\nset(gca,'LineStyleOrder', '--');";
    // matlabFile << "\ntext(1,0.685 ,'Threshold' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex');";

    matlabFile << "\nglobalF1= [";
    
    std::string value;
    std::ifstream ReadGlobalF1 (global_F1_vs_learned_category.c_str());
    std::getline (ReadGlobalF1, value);
    matlabFile << value;

    int iteration = 1;
    while (ReadGlobalF1.good())
    {
		std::getline (ReadGlobalF1, value);
		if(value.empty ()) // Skip blank lines or comments
			continue;
		//  ROS_INFO("value = %s", value.c_str());
		matlabFile <<", " << value ;
		iteration++;
    }
    
    int remain = iteration % 5;
    matlabFile << "];\naxis([0,"<< iteration + 5 - remain <<",0.5,1.05]);"; 
    matlabFile << "\nplot(1:size(globalF1,2), globalF1(1:size(globalF1,2)),'-.O', 'Color',[0 0 1], 'LineWidth',2.);";
    matlabFile << "\nxlabel('Number of Learned Categories','FontSize',15);";
    matlabFile << "\nylabel('Global Classification Accuracy','FontSize',15);";
    matlabFile.close();
    ROS_INFO("\t\tMATLAB file created...");
}


/////////////////////////////////////////////////////////////////////////////////////////////
void plotNumberOfLearnedCategoriesVsIterationsInMatlab( int RunCount, 
														string Number_of_learned_categories_vs_Iterations)
{
 
    // If the text inside the plot is not well-appear, please install the font in ubuntu using the following commands.
    // sudo apt-get install xfonts-75dpi
    // sudo apt-get install xfonts-75dpi
    
    char run_count [10];
    sprintf( run_count, "%d",RunCount );
    string path = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/number_of_learned_categories_vs_Iterations.m";
        
    ROS_INFO("\t\tpath = %s", path.c_str());

    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "figure ();\nhold on;\ngrid on;";
    matlabFile << "\nset(gca,'LineStyleOrder', '-');";
    matlabFile << "\nNLI= [";
    
    int iteration = 1;
    std::string value;
    std::ifstream ReadNLI (Number_of_learned_categories_vs_Iterations.c_str());
    std::getline (ReadNLI, value);
    matlabFile << iteration;

    while (ReadNLI.good())
    {
	std::getline (ReadNLI, value);
	if(value.empty ()) // Skip blank lines or comments
	    continue;	
	if (strcmp(value.c_str(),"1")==0)
	{
	    matlabFile << "," << iteration ;	    
	    iteration++;
	}
	else
	{
	    iteration++;
	}
    }
    matlabFile << "];";
    matlabFile << "\nplot(NLI(1:size(NLI,2)), 1:size(NLI,2), '--O', 'Color',[1 0 0], 'LineWidth',2.)";
    matlabFile << "\nxlabel('Question / Correction Iterations','FontSize',15);";
    matlabFile << "\nylabel('Number of Learned Categories','FontSize',15);";
    matlabFile.close();
    ROS_INFO("\t\tMATLAB file created...");
}

/////////////////////////////////////////////////////////////////////////////////////////////
void plotNumberOfStoredInstancesPerCategoryInMatlab( vector <ObjectCategory> list_of_object_category)
{
 
	string path = ros::package::getPath("simulated_teacher")+ "/result/experiment_1/number_of_stored_instaces_per_category.m";
    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "figure ();\nhold on;\ngrid on;";
    matlabFile << "\nNIC= [";
	
	string path_tmp = home_directory_address+ "/Category/Category.txt";
    std::ifstream category_name (path_tmp.c_str());
    string cat_name ="";

	int counter = 0;
	while ((category_name.good()) && 
			(counter < list_of_object_category.size()))
    {
		int idx = 0;
		std::getline (category_name, cat_name);
 	   	cat_name = extractCategoryName(cat_name);
		while ((list_of_object_category.at(idx).cat_name.c_str() != cat_name) && 
		 		(idx < list_of_object_category.size()))
		{
			idx ++;
		}

		if (counter < list_of_object_category.size()-1)
		{
			matlabFile << list_of_object_category.at(idx).rtov_keys.size()<<",";
		}
		else
		{
			matlabFile << list_of_object_category.at(idx).rtov_keys.size()<<"];\n";
		}		
		counter ++;
	}
	
	matlabFile << "list = {'";
	std::ifstream category_name_tmp (path_tmp.c_str());
	for (size_t i = 0; i < list_of_object_category.size(); i++) // retrieves all categories from perceptual memory
	{
		int idx = 0;
		std::getline (category_name_tmp, cat_name);
 	   	cat_name = extractCategoryName(cat_name);
		int lfind =  cat_name.find("_");
    	if (lfind > 0) 
			cat_name.replace(lfind, 1,1, '-');

		if (i < list_of_object_category.size()-1)
		{
			matlabFile << cat_name<<"', '";
		}
		else
		{
			matlabFile << cat_name<<"'};\n";
		}			
	}
	matlabFile << "categories = categorical(list,list);\n";
	matlabFile << "bar (categories, NIC);\n";
    matlabFile << "ylabel('Number of Stored Instances','FontSize',15);";
    matlabFile.close();
    ROS_INFO("\t\tMATLAB file created...");

    ROS_INFO("\t\t cleaning log files...");

	// string system_command= "rm " + ros::package::getPath("simulated_teacher") + "/result/experiment_1/Category.txt";
	// system( system_command.c_str());

	string system_command= "rm " + ros::package::getPath("simulated_teacher") + "/result/experiment_1/Category_Introduced.txt";
	system( system_command.c_str());
	
	system_command= "rm " + ros::package::getPath("simulated_teacher") + "/result/experiment_1/f1_vs_learned_category.txt";
	system( system_command.c_str());

	system_command= "rm " + ros::package::getPath("simulated_teacher") + "/result/experiment_1/local_f1_vs_learned_category.txt";
	system( system_command.c_str());

	system_command= "rm " + ros::package::getPath("simulated_teacher") + "/result/experiment_1/PrecisionMonitor.txt";
	system( system_command.c_str());

}

/////////////////////////////////////////////////////////////////////////////////////////////
bool fexists(std::string filename) 
{
  ifstream ifile(filename.c_str());
  return ifile.good();
}

/////////////////////////////////////////////////////////////////////////////////////////////
int sum_all_experiments_results ( int iterations,
				  float Success_Precision		,			    
				  float average_class_precision_value,
				  float number_of_stored_instances, 
				  int number_of_taught_categories,
				  string name_of_approach
				  )
{
    vector <float> tmp;
    tmp.push_back(float (iterations)); 
    tmp.push_back(float (number_of_taught_categories)); 
    tmp.push_back(float (number_of_stored_instances)); 
    tmp.push_back(Success_Precision); 
    tmp.push_back(average_class_precision_value); 

    std::string sumResultsOfExperiments;
    sumResultsOfExperiments = ros::package::getPath("simulated_teacher")+ "/result/sum_all_results_of_"+name_of_approach+"_experiments.txt";
    ROS_INFO("results of expriments file path = %s",sumResultsOfExperiments.c_str() );
    int exp_num =1;
    if (!fexists(sumResultsOfExperiments.c_str()))
    {
		ROS_INFO("File not exist");	
		std::ofstream sum_results_of_experiments;
		sum_results_of_experiments.open (sumResultsOfExperiments.c_str(), std::ofstream::out);
		sum_results_of_experiments.precision(4);
		sum_results_of_experiments <<iterations <<"\n"<< number_of_taught_categories <<"\n"<< number_of_stored_instances/(float) number_of_taught_categories <<"\n"<< Success_Precision<< "\n"<< average_class_precision_value;
		sum_results_of_experiments.close();
    }
	else
    {
		ROS_INFO("File exist");
		string tmp_value;
		std::ifstream results_of_experiments;
		results_of_experiments.open (sumResultsOfExperiments.c_str());
		vector <float> value;
		for (int i=0; (results_of_experiments.good()) && (i < tmp.size()) ; i++)
		{
			std::getline (results_of_experiments, tmp_value);
			if(tmp_value.empty () || tmp_value.at (0) == '#') // Skip blank lines or comments
			continue;
			value.push_back(atof(tmp_value.c_str())+tmp.at(i));
		    // ROS_INFO("tmp_value.c_str() = %s",tmp_value.c_str() );
		    // ROS_INFO("atof -> tmp_value.c_str() = %f",atof(tmp_value.c_str()) );
		    // ROS_INFO("tmp[%i] = %f",i, tmp.at(i) );
		    // ROS_INFO("value[%i]= %f",i, value.at(i) );			
		}
		results_of_experiments.close();
		std::ofstream sum_results_of_experiments;
		sum_results_of_experiments.open (sumResultsOfExperiments.c_str(), std::ofstream::out);
		sum_results_of_experiments.precision(4);
		sum_results_of_experiments <<value.at(0) <<"\n"<< value.at(1) <<"\n"<< value.at(2) <<"\n"<< value.at(3)<< "\n"<< value.at(4);
		sum_results_of_experiments.close();
    } 

	return 0;	
    
}
/////////////////////////////////////////////////////////////////////////////////////////////
int average_all_experiments_results ( int total_number_of_experiments,
				      string name_of_approach)
{
    std::string sumResultsOfExperiments;
    sumResultsOfExperiments = ros::package::getPath("simulated_teacher")+ "/result/sum_all_results_of_"+name_of_approach+"_experiments.txt";
    ROS_INFO("results of expriments file path = %s",sumResultsOfExperiments.c_str() );
    
    if (!fexists(sumResultsOfExperiments.c_str()))
    {
	ROS_INFO("Expriments summary file of not exist");	
    }else
    {
	ROS_INFO("File exist");
	string tmp_value;
	std::ifstream read_results_of_experiments;
	read_results_of_experiments.open (sumResultsOfExperiments.c_str());
	vector <float> value;
	for (int i=0; read_results_of_experiments.good(); i++)
	{
	    std::getline (read_results_of_experiments, tmp_value);
	    if(tmp_value.empty () || tmp_value.at (0) == '#') // Skip blank lines or comments
		continue;
	    value.push_back(atof(tmp_value.c_str())/total_number_of_experiments);	    
	}
	read_results_of_experiments.close();

	std::string resultsOfExperiments;
	resultsOfExperiments = ros::package::getPath("simulated_teacher")+ "/result/results_of_"+name_of_approach+"_experiments.txt";
	std::ofstream results_of_experiments;
	results_of_experiments.open (resultsOfExperiments.c_str(), std::ofstream::app);
	results_of_experiments.precision(4);
	results_of_experiments << "\n"<<"Avg."<<"\t"<<value.at(0) <<"\t\t"<< value.at(1) <<"\t\t"<< value.at(2) <<"\t\t"<< value.at(3)<< "\t\t"<< value.at(4);
	results_of_experiments << "\n---------------------------------------------------------------------------";
	results_of_experiments.close();
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
int reportAllExperimentalResults (int TP, int FP, int FN,int obj_num,			    
									vector <float> average_class_precision,
									float number_of_stored_instances, 
									int number_of_taught_categories,
									string name_of_approach
								)
{
    //ROS_INFO("TEST report_all_experiments_results fucntion");
    int total_number_of_experiments = 50;//TODO: shold be input parameres
    unsigned int number_of_exist_experiments = 0;
    
    float average_class_precision_value = 0;
    for (int i = 0; i < average_class_precision.size(); i++)
    {
		average_class_precision_value += average_class_precision.at(i); 	    
    }
    average_class_precision_value = average_class_precision_value / average_class_precision.size();
	
    std::string resultsOfExperiments;
    resultsOfExperiments = ros::package::getPath("simulated_teacher")+ "/result/results_of_"+name_of_approach+"_experiments.txt";
    ROS_INFO("results of expriments file path = %s",resultsOfExperiments.c_str() );
    int exp_num =1;
    double Success_Precision = 0;
    if (!fexists(resultsOfExperiments.c_str()))
    {
	ROS_INFO("File not exist");
	Success_Precision = TP/double (TP+FP);
	double Recall = TP/double (TP+FN);

	std::ofstream results_of_experiments;
	results_of_experiments.open (resultsOfExperiments.c_str(), std::ofstream::out);
	results_of_experiments.precision(4);
	results_of_experiments << "\nNum"<<"\tIterations" <<"\t"<< "Categories" <<"\t"<< "Instances"<< "\t"<< "GS" << "\t\t"<< "ACS";
	results_of_experiments << "\n---------------------------------------------------------------------------";
	results_of_experiments << "\n"<<exp_num<<"\t"<<obj_num <<"\t\t"<< number_of_taught_categories <<"\t\t"<< number_of_stored_instances/(float)number_of_taught_categories <<"\t\t"<< Success_Precision<< "\t\t"<< average_class_precision_value;
	results_of_experiments << "\n---------------------------------------------------------------------------";
	results_of_experiments.close();
	results_of_experiments.clear();
    }else
    {
	ROS_INFO("File exist");
	string tmp;
	std::ifstream num_results_of_experiments;
	num_results_of_experiments.open (resultsOfExperiments.c_str());
	while (num_results_of_experiments.good()) 
	{
	    std::getline (num_results_of_experiments, tmp);
	    if(tmp.empty () || tmp.at (0) == '#') // Skip blank lines or comments
		continue;
	    number_of_exist_experiments++;
	}
	num_results_of_experiments.close();
	ROS_INFO("number_of_exist_experiments = %i",number_of_exist_experiments );
	exp_num = (number_of_exist_experiments)/2;
	ROS_INFO("(number_of_exist_experiments)/2 = %i",exp_num );

	Success_Precision = TP/double (TP+FP);
	double Recall = TP/double (TP+FN);
	std::ofstream results_of_experiments;
	results_of_experiments.open (resultsOfExperiments.c_str(), std::ofstream::app);
	results_of_experiments.precision(4);
	results_of_experiments << "\n"<<exp_num<<"\t"<<obj_num <<"\t\t"<< number_of_taught_categories <<"\t\t"<< number_of_stored_instances/(float)number_of_taught_categories <<"\t\t"<< Success_Precision<< "\t\t"<< average_class_precision_value;
	results_of_experiments << "\n---------------------------------------------------------------------------";
	results_of_experiments.close();
    } 
    
    sum_all_experiments_results ( obj_num,
				  Success_Precision,			    
				  average_class_precision_value,
				  number_of_stored_instances, 
				  number_of_taught_categories,
				  name_of_approach );

    if (exp_num == total_number_of_experiments)
    {
		average_all_experiments_results (total_number_of_experiments, name_of_approach);
    }
    
	return 0;	
}



