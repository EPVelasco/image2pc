#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <std_msgs/Header.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>

#include <chrono>

#include <sensor_msgs/PointCloud2.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

#include <math.h>

using namespace Eigen;
using namespace sensor_msgs;
using namespace std;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub_1;

float max_depth =100.0;
float min_depth = 0.0;
std::string imgTopic = "/depht_image";

int total_frame=0;
float time_delay  = 0;
double total_time =0;

void callback(const ImageConstPtr& imgIn)
{

  std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();


    cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(imgIn, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  cv::Mat img  = cv_ptr->image;  

  int cols = img.cols;
  int rows = img.rows;

  cv::Rect roi;
  roi.x = 0;
  roi.y = 0;
  roi.width = cols/2;
  roi.height = rows;
  cv::Mat l_img = img(roi);
  
  roi.x = cols/2 ;
  cv::Mat z_img = img(roi);

  Eigen::Matrix<float,Dynamic,Dynamic> z_data;
  Eigen::Matrix<float,Dynamic,Dynamic> z_range = Eigen::Matrix<float,Dynamic,Dynamic>::Zero(rows, cols/2);;
  cv2eigen(z_img,z_data); 

  for (int i=0; i< z_data.rows(); ++i)
   {       
      for (int j=0; j<z_data.cols() ; ++j)
      {
        if (z_data(i,j) == 0)
          continue; 

        z_range(i,j) = (100.0/pow(2,16))*(z_data(i,j)) -50.0; 
      }
   }
   
  cv::Mat img_frame = l_img.clone();
 // img_frame.convertTo(img_frame, CV_8UC1, 1 / 256.0);

  cv::Mat ground_mask_inv;
  cv::Mat curr_surf = l_img.clone();
  Eigen::Matrix<float,Dynamic,Dynamic> img_frame_eig;
  cv2eigen(img_frame,img_frame_eig);


  //pcl::PointCloud<pcl::PointXYZ> edge_cloud;
  PointCloud::Ptr output_pc (new PointCloud);
  output_pc->width = l_img.cols; 
  output_pc->height = l_img.rows;
  output_pc ->is_dense = false;
  output_pc->points.resize (output_pc->width * output_pc->height);

  int num_pc =0;
  for (int i=0; i< l_img.rows; i+=1)
   {       
      for (int j=0; j<l_img.cols ; j+=1)
      {
        float ang= M_PI - ((2.0 * M_PI * j )/(l_img.cols));
        if(!(img_frame_eig(i,j)== 0)){

          float img_data = (pow(2,16)-img_frame_eig(i,j))*(max_depth)/pow(2,16);                
          float pc_x_data = sqrt(pow(img_data,2)- pow(z_range(i,j),2)) * cos(ang);
          float pc_y_data = sqrt(pow(img_data,2)- pow(z_range(i,j),2)) * sin(ang);
          output_pc->points[num_pc].x = pc_x_data;
          output_pc->points[num_pc].y = pc_y_data;
          output_pc->points[num_pc].z = z_range(i,j);
          num_pc++;

        }
      }
   }   

  output_pc->header.frame_id = "velodyne";
  ros::Time time_st = imgIn->header.stamp; 
  output_pc->header.stamp    =  time_st.toNSec()/1e3;
  pub_1.publish (output_pc);                

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image2pc");
  ros::NodeHandle nh;  

  /// Load Parameters
  nh.getParam("/max_depth", max_depth);
  nh.getParam("/imgTopic", imgTopic);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/depth_image", 10, callback);
  pub_1= nh.advertise<PointCloud> ("/pc_output", 10);
  ros::spin();

}
