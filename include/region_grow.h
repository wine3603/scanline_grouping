/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % 2016 01 22 @ the Univ. of Tokyo
  % Tianwei Zhang
  % file: region_grow.h
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#ifndef REGION_GROW_H
#define    REGION_GROW_H

#include<pcl/common/time.h>
#include<pcl/pcl_base.h>
#include<pcl/point_cloud.h>
#include<math.h>
#include<time.h>
#include<pcl/point_types.h>
#include<list>
#include<pcl/common/eigen.h>
//using namespace Eigen;
namespace region_growing{
	typedef pcl::PointXYZI PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef struct {PointT end_point; 
		size_t index;
		PointT end_point_e;
		size_t index_e;
		size_t length;
		PointT vector;
		size_t height;} split_point;
	typedef std::vector <split_point> split_line;
	typedef std::vector <split_line> split_image;
	typedef struct { float RMS;
			 size_t size; //number of line segs
			 //Eigen::Vector3f n;
			split_line lines;
			PointT n;
			PointT m;
			 float d;} Plane;//save planes para
	typedef std::vector <Plane> Planes;
	typedef std::map< int, split_line> m_plane;
	typedef  std::vector<m_plane> mer_plane;
	/////save vector map 
	static split_image image_helper;
	/////save plane seg candidant
	static split_line patch;
	////save plane maps
	static split_image patch_map;
	static split_image merge_map;
	////plane parameters d and n(vector)
	static float dis_plane; 
	static PointT n;
	static size_t min_size = 5;
	/// min seeding line distance and length
//	static float seed_len = 0.2;
//	static float seed_dis = 1.0;
	static bool break_sign = false;
	///plane fitting threshod
	static float theta_grow = 0.03;
	static float theta_seed = 0.02;
	static PointCloudT plane_cloud;
	static float RMS;
	static PointT m_;
	static Planes planes;
	static PointCloudT normals;
static int patch_no, patchmap_no, comtru_no, extend_, seeds_;
	class region_grow { 
		public:
			split_image grow(split_image &image_vec);
			PointCloudT get_normals();
			split_image merge_plane();
			inline float sq(float x) {return x * x;}
		private:
			////generate vextor map and length for input 
			bool  prepare_growing (split_image &image) ;
			////get data from split pt struct
			split_point get_data(split_line::iterator &itr);
			///comp if a b are coplanar
			bool compare(split_line::iterator & a, split_line::iterator &b);
			/////compute plane formular A*m = b /2016/02/16
			bool coplanar_or_not(split_line::iterator &c);
			bool coplanar_or_not(split_line::iterator &a,split_line::iterator &b,split_line::iterator &c);
			//////add a b to same plane candidante
			void add_into_patch(split_line::iterator &a);
			////comput plane formular, return vec_m, n is nor(m), d is reciprocal(m), input is the point cloud of query pt
			bool  plane_formular(PointCloudT &sigma);
			bool  recom_plane(PointCloudT &sigma);
			float det(PointT &a, PointT &b, PointT  &c);	
			void extend (split_line::iterator &itr);				
			void seeding ();				
			void make_patch(split_line::iterator &a,split_line::iterator &b, split_line::iterator &c);			
			bool a_is_in_patch(split_line::iterator &a);
			PointT minus ( PointT &a, PointT &b);
			///martrix product 3x3 return in R1
			float mar_mul_R1(PointT &n_3, PointT &b_3 );
		public: 

		//	split_image::iterator extend_line;
			split_image::iterator query_line;
			split_line::iterator query_pt_itr;
		//	split_line::iterator extend_pt_itr;





	};
}
#endif
