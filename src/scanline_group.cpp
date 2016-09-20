/*******************************************************************
 * Copyright (c) 2015 12 26 the Univ of Tokyo
 *
 * @file scanlien_group.cpp
 * @brief laser line split
 * @author Tianwei Zhang
 *******************************************************************/
#include <ros/ros.h>
//#include <pcl_conversions/pcl_conversions.h> //hydro
#include <sensor_msgs/PointCloud2.h> //hydro
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
//#include <fstream>
//#include <iostream>

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <vtkPolyLine.h>

//#include <omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <limits>
#include <rdpl.h>
#include <region_grow.h>

#include <geometry_msgs/PoseArray.h>
// Types

using namespace std;
static PointCloudT  sPCLCloud;
static PointCloudT SplitOut;
static PointCloudT lineCloud;
static split_image Split_Out;
class scan_group
{
	public:
		scan_group()
		{
			// Create ROS subscriber for the input point cloud
			// sensor_g_sub = n.subscribe("HRP4/sensor/g",10, &sv_filter::g_cb, this);
			scanline_sub = n.subscribe ("/zhang_Point_Cloud", 100, &scan_group::pcd_cb, this);

			// Create a ROS publisher for the output point cloud
			edge_pcd_pub = n.advertise<sensor_msgs::PointCloud2> ("edge_pcd", 100);
			color_plane_pub = n.advertise<sensor_msgs::PointCloud2> ("color_plane", 100);
//			plane_nor_pub = n.advertise<geometry_msgs::PoseArray> ("plane_normals", 100);
			merge_pub = n.advertise<sensor_msgs::PointCloud2> ("merge_planes", 100);
		}

	private:
		void pcd_cb (const PointCloudT & pcd){
			//std::cout<<"height..."<<pcd.height<<"...width...."<<pcd.width<<std::endl;
			Split_Out.clear();
			std::cout<<"pcd height :"<<pcd.height<<std::endl;
			for (int r = 0; r < pcd.height; r++){//row no
				for (int c = 0; c< pcd.width; c++){//column no.
					lineCloud.points.push_back(pcd.at(c,r));
				}
				double epsilon = 0.02;//TODO adaptive
				split_line edge_pt;
				rdpl rdpl_im;
				rdpl_im.rdp_implimentation(lineCloud, edge_pt, epsilon);
				lineCloud.clear();
				split_line::iterator itr = edge_pt.begin();
				for(; itr!= edge_pt.end();itr++){
					itr->height = r;
				}
				Split_Out.push_back(edge_pt);
				//	std::cout<<Split_Out.size()<<std::endl;
			}
/*
PointCloudT Minus_Edge;
for(int r = 0; r< pcd.height;r++){
 for(int c = 0; c< pcd.width; c++){
  if (c >0){
	if(fabs(pcd.at(c,r).intensity - pcd.at(c-1,r).intensity)> 0.034364*pcd.at(c-1,r).intensity)
	Minus_Edge.points.push_back(pcd.at(c,r));
	}
    }
}
std::cout<< "size of Minus edge "<< Minus_Edge.size()<<std::endl;
*/
			frame_ = "dynamixel_base";
			PointCloudT edge_cloud;
			edge_cloud.header.frame_id = frame_;
			std::vector<split_line>::iterator height_itr = Split_Out.begin(); 
			for (; height_itr< Split_Out.end(); height_itr++){
				std::vector<split_point>::iterator width_itr = height_itr->begin();
				for (; width_itr< height_itr->end(); width_itr++){
					edge_cloud.push_back(width_itr->end_point);
				}	
			}
	 
			region_growing::region_grow reg_grow;
			split_image planes = reg_grow.grow(Split_Out);
			pcl::PointCloud<pcl::PointXYZRGB> color_plane;
			pcl::PointCloud<pcl::PointXYZRGB> merge_cloud;
			color_plane.header.frame_id = frame_;
			split_image::iterator imag_itr = planes.begin();
			for(;imag_itr != planes.end();imag_itr++){
				split_line::iterator line_itr = imag_itr->begin();
				pcl::PointXYZRGB pt(rand()%256, rand()%256, rand()%256);
				//pcl::PointXYZRGB pt(0, 255, 0);
				for(;line_itr != imag_itr->end();line_itr++){
					size_t c = line_itr->index;
					int r = line_itr->height;
					//		std::cout<<" heigh "<< r <<" length " <<line_itr->length;
					for(int i = 0; i < line_itr->length;i++){
						pt.x = pcd.at(c+i,r).x;
						pt.y = pcd.at(c+i,r).y;
						pt.z = pcd.at(c+i,r).z;
						color_plane.points.push_back(pt);
					}
					//	std::cout<<"c,r  " <<r<<", height "<<line_itr->height<<std::endl;
				}
			}

			split_image merge_plane = reg_grow.merge_plane();
//		std::cout<<" merge plane no. " << merge_plane.size()<<std::endl;
			split_image::iterator p_itr = merge_plane.begin();
			merge_cloud.header.frame_id = frame_;
			for(;p_itr != merge_plane.end();p_itr++){
				split_line::iterator line_itr = p_itr->begin();
				pcl::PointXYZRGB pt(rand()%256, rand()%256, rand()%256);
				//pcl::PointXYZRGB pt(0, 255, 0);
				for(;line_itr != p_itr->end();line_itr++){
					size_t c = line_itr->index;
					int r = line_itr->height;
					//		std::cout<<" heigh "<< r <<" length " <<line_itr->length;
					for(int i = 0; i < line_itr->length;i++){
						pt.x = pcd.at(c+i,r).x;
						pt.y = pcd.at(c+i,r).y;
						pt.z = pcd.at(c+i,r).z;
						merge_cloud.points.push_back(pt);
					}
					//	std::cout<<"c,r  " <<r<<", height "<<line_itr->height<<std::endl;
				}
			}

			edge_pcd_pub.publish(edge_cloud);
			color_plane_pub.publish(color_plane);
			merge_pub.publish(merge_cloud);
			std::cout<<"edge pt no. :"<<edge_cloud.size()<<std::endl;
			edge_cloud.clear();
			color_plane.clear();
			//	normals.clear();
		}

		ros::NodeHandle n;
		ros::Publisher  edge_pcd_pub;
		ros::Publisher  merge_pub;
		ros::Publisher  color_plane_pub;
//		ros::Publisher  plane_nor_pub;
		ros::Subscriber scanline_sub;
		//ros::Subscriber sensor_g_sub;	

		geometry_msgs::PoseArray normal;
		std::string frame_ ;
};

	int
main (int argc, char** argv)
{
	ros::init (argc, argv, "scanline_group_node");

	scan_group scan_group;

	ros::spin ();
	return (0);
}
