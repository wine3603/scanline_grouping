/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % 2016 01 22 the university of tokyo
  % author Tianwei Zhang
  % region growing of scanline grouping
  % filename: region_grow.cpp
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include<region_grow.h>
using namespace region_growing;
using namespace std;
using namespace Eigen;

bool region_grow::prepare_growing(split_image &image){//init vector of each line segment
	if(!image.size())
		return false;
	image_helper = image;
	split_image::iterator height_itr = image_helper.begin();
	patch.clear();
	patch_map.clear();
	merge_map.clear();
	planes.clear();
	for (; height_itr< image_helper.end(); height_itr++){
		split_line::iterator width_itr = height_itr->begin();
		for (; width_itr< height_itr->end() - 1; width_itr++){
			split_line::iterator temp_itr = width_itr + 1;
			PointT temp1 =  width_itr->end_point ;
			PointT temp2 = temp_itr->end_point;
			temp1.x -= temp2.x;
			temp1.y -= temp2.y;
			temp1.z -= temp2.z;
			width_itr->vector = temp1;
			width_itr->length = temp_itr->index - width_itr->index;
			//std::cout<<"height "<<width_itr->height<<"  length  " <<width_itr->length<<" index "<<width_itr->index<<std::endl;
		}
		//++height;	
	}
	if (image_helper.size() != 0){
		//std::cout<<"patch_map size "<<patch_map.size()<<std::endl;
		seeds_ = 	patch_no  = extend_= patchmap_no = comtru_no = 0;
		normals.clear();
		return true;
	}
}

split_image region_grow::grow (split_image &image_vec){
	if(!(prepare_growing(image_vec))){
		std::cout<<" input error ";
		//return;
	}
	else{
		double seed_start = pcl::getTime();
		seeding();
		double seed_over = pcl::getTime();
		std::cout<<"||||seeding time is ......."<<seed_over - seed_start<<std::endl;
	}
	if(patch.size()){
		patch_map.push_back(patch);
		planes.back().lines = patch_map.back();
	}
	int im_size = 0;
	if(image_helper.size()!=0){
		split_image::iterator teitr = image_helper.begin();
		for(;teitr != image_helper.end();teitr++)
		{
			split_line::iterator a_itr = teitr->begin();
			for(;a_itr != teitr->end(); a_itr++)
			{
				im_size++;
			}}}
	std::cout<< "add in patch no "<<patch_no<< " map no " <<patchmap_no<<" seeds "<<seeds_<<" image size after seeding  "<< im_size <<std::endl;
	std::cout<<"patch_map size "<<patch_map.size()<<std::endl;
	std::cout<<"planes size "<<planes.size()<<std::endl;
for(int i = 0; i < planes.size();i++){
//	std::cout<<" plane :"<<  i << " d "<<planes[i].d<<" n "<<planes[i].n<<std::endl;
}	//merge_plane();
//	std::cout<<" after merging " << planes.size()<< " merged  numbers "<< comtru_no<<std::endl;
	return(patch_map);//output the planes map
}

void region_grow::seeding(){
	query_line = image_helper.begin();
	for (; query_line != image_helper.end()-3; query_line++){ 
		query_pt_itr = query_line->begin(); 
		split_image::iterator line_1 = query_line +1; 
		split_image::iterator line_2 = query_line +2;
		bool sign = false;
		double line1_start = pcl::getTime();
		for(; query_pt_itr !=  query_line->end();){
			double line2_start = pcl::getTime();
			for (split_line::iterator itr_1 = line_1->begin();
					itr_1 != line_1 ->end();){
				if(compare(query_pt_itr, itr_1)){
					split_line::iterator itr_2 = line_2->begin();
					double line3_start = pcl::getTime();
					for(; itr_2 != line_2->end();){
						double com_start = pcl::getTime();
						if(compare(itr_1, itr_2)){
							if (coplanar_or_not(query_pt_itr, itr_1, itr_2))
							{ seeds_++;
								sign = true;
//	static std::map< int, split_line> m_plane;
//	static std::vector<m_plane> mer_plane;
								make_patch(query_pt_itr, itr_1, itr_2);
								extend(itr_2);
								line_2->erase(itr_2++);
								line_1->erase(itr_1++);
								query_line->erase(query_pt_itr++);
								break;
							}
							else
								++itr_2;
							double com_over = pcl::getTime();
						}
						else if(break_sign){
							break_sign = false;
							break;}
						else ++itr_2;
						//std::cout<<"each com ......:"<<	com_over-com_start<<std::endl;
					}	
					double line3_over = pcl::getTime(); 
					//std::cout<<"line3 over........"<<line3_over- line3_start<<std::endl;
					if(sign){
						break;}
					else
						++itr_1;
				}
				else if(break_sign){
					break_sign = false;
					break;}
				else ++itr_1;
			}
			if (sign){
				sign = false;
			}
			else ++query_pt_itr;
			double line2_over = pcl::getTime();
			//std::cout<<"line2 loop .... "<<line2_over - line2_start<<std::endl;
		}
		double line1_over = pcl::getTime();
		//std::cout<<"line1 loop .... "<<line1_over - line1_start<<std::endl;
	}
}

void region_grow::extend(split_line::iterator &itr){
	extend_++;
	//	split_image::iterator	extend_line = line +1 ;
	split_point pt = get_data(itr);
	if (pt.height<2 ||pt.height>17000)
		std::cout<<"  get data error when extending: height =  "<<pt.height ;	
	if (pt.length < 3)
	return ;
	split_image::iterator extend_line = image_helper.begin() + (pt.height)+ 1;
	if( extend_line == image_helper.end()){
		//	query_line->erase(query_pt_itr++);
		return;
	}
	split_line::iterator e_itr = extend_line->begin();
	for (;e_itr != (extend_line)->end();){
		if(compare(itr, e_itr)){
			if(coplanar_or_not(e_itr)){
				add_into_patch(e_itr);
				extend(e_itr);
				extend_line->erase(e_itr++);//		std::cout<<"3 ";
				//	break;
				
			}

			else
				++e_itr;
		}
		else if (break_sign){
			break_sign = false;
			break;}
		else  ++e_itr;
	}
}

split_point region_grow::get_data(split_line::iterator &itr){
	split_point temp;
	temp.end_point = itr->end_point;
	temp.index = itr->index;
	temp.length = itr->length;
	temp.vector = itr->vector;
	temp.height = itr->height;
	return temp;
}

/////////////compare the query points' index, if far, break.
bool region_grow::compare(split_line::iterator &a_itr, split_line::iterator &b_itr){
	split_point a = get_data(a_itr);
	split_point b = get_data(b_itr);
	if(a.length<min_size || b.length<min_size)
	return false;
	if (b.index >= (a.index + a.length)) 
	{
		break_sign = true;
		return false;
	}
	else if((b.index + b.length) <= a.index)
		return false;
	return true;
}

bool region_grow::coplanar_or_not (split_line::iterator &c){
	split_point x = get_data(c);
	PointT s, e;
//	std::cout<< c-> index<<std::endl;
	s.x = c->end_point.x;
	s.y = c->end_point.y;
	s.z = c->end_point.z;
	e.x = s.x - c->vector.x;
	e.y = s.y - c->vector.y;
	e.z = s.z - c->vector.z;

	size_t N  = plane_cloud.size();
//std::cout<< " //siz " << N << std::endl;
	float  RMS_grow = sqrt((sq(s.x*n.x + s.y*n.y + s.z*n.z - dis_plane ) + sq(e.x*n.x + e.y*n.y + e.z*n.z - dis_plane) )*0.5f);
//	float  RMS_grow = sqrt((N * sq(RMS) + sq(s.x*n.x + s.y*n.y + s.z*n.z - dis_plane ) + sq(e.x*n.x + e.y*n.y + e.z*n.z - dis_plane) )/(N+2));

//std::cout<<" RMS_grow  "<<RMS_grow <<std::endl;
	if(RMS_grow < theta_grow){
		plane_cloud.points.push_back(s);
		plane_cloud.points.push_back(e);
	return	recom_plane(plane_cloud);
		//update plane n , d
//		return	true;
	}
		return false;
}

bool region_grow::coplanar_or_not (split_line::iterator &a, split_line::iterator &b, split_line::iterator &c){
double cop_start = pcl::getTime();
	split_point x = get_data(a);
	split_point y = get_data(b);
	split_point z = get_data(c);
//check the length of seeding lines
//	if((x.length<10)||(y.length<10)||(z.length<10)){
//std::cout<<".. length  f.. ";
//	return false;
//	}
//	if((sqrt(sq(x.vector.x) + sq(x.vector.y) + sq(x.vector.z))< seed_len)||(sqrt(sq(y.vector.x) + sq(y.vector.y) + sq(y.vector.z))< seed_len)||(sqrt(sq(z.vector.x) + sq(z.vector.y) + sq(z.vector.z))< seed_len))
//	{
//std::cout<<"..vec_dis  f.. ";
//	return false;
//}
	PointT Va,Vb,Vc,Vab,Vac;
	Va = x.vector;
	Vb = y.vector;
	Vc = z.vector;
	Vab = minus(x.end_point, y.end_point);
	Vac = minus(x.end_point, z.end_point);
////check distance between lines
//	if((sqrt(powf(Vab.x, 2.0) + powf(Vab.y, 2.0) + powf(Vab.z, 2.0))> seed_dis)||( sqrt(powf(Vac.x, 2.0) + powf(Vac.y, 2.0) + powf(Vac.z, 2.0))> seed_dis)){
// std::cout<<"..line between  f.. ";
//		return false;
//}
//	if (std::abs(det(Va, Vb, Vab)) < theta_){//TODO
		PointT Pa, Pa_, Pb, Pb_, Pc, Pc_;
		Pa = x.end_point;
		Pb = y.end_point; 
		Pc = z.end_point;
		Pa_ = minus(Pa, x.vector);
		Pb_ = minus(Pb, y.vector);
		Pc_ = minus(Pc, z.vector);
		PointCloudT Sigema_A;
		Sigema_A.clear();
		Sigema_A.points.push_back(Pa);
		Sigema_A.points.push_back(Pb);
		Sigema_A.points.push_back(Pc);
		Sigema_A.points.push_back(Pa_);
		Sigema_A.points.push_back(Pb_);
		Sigema_A.points.push_back(Pc_);
//	std::cout << "distanc from l1 to l2 "<<sqrt(powf(Vab.x, 2.0) + powf(Vab.y, 2.0) + powf(Vab.z, 2.0))<<std::endl;
	double cop_over = pcl::getTime();
//	std::cout<<"cop time ................"<<cop_over-cop_start<<std::endl;
		return plane_formular(Sigema_A);
//	}
}

bool region_grow::plane_formular(PointCloudT &sigema){
////least-square fit the plane by Root Mean Square
	double rms_t = pcl::getTime();
	
Matrix<float, 6, 4> SVD_A;

SVD_A  <<
sigema.points[0].x, sigema.points[0].y, sigema.points[0].z, -1.0 ,
sigema.points[1].x, sigema.points[1].y, sigema.points[1].z, -1.0 ,
sigema.points[2].x, sigema.points[2].y, sigema.points[2].z, -1.0 ,
sigema.points[3].x, sigema.points[3].y, sigema.points[3].z, -1.0 ,
sigema.points[4].x, sigema.points[4].y, sigema.points[4].z, -1.0 ,
sigema.points[5].x, sigema.points[5].y, sigema.points[5].z, -1.0 ;
JacobiSVD<MatrixXf> svd(SVD_A, ComputeThinU | ComputeThinV);

	Matrix<float, 3,1> n_, pi, n_svd, n_svd_norm;
	Matrix3f M;
//	n_<< n.x, n.y, n.z;
//	m_ = m;

Matrix <float, 4, 4> m_v = svd.matrixV();
n_svd<< m_v(0,3),m_v(1,3), m_v(2,3);
float dis_svd =  m_v(3, 3);
dis_svd /= n_svd.norm();
n_svd.normalize();
if (dis_svd < 0){
dis_svd *= -1;
n_svd *= -1;
}
//std::cout << "Its right singular vectors are the columns of the thin V matrix:" << n_svd <<" "<< dis_svd << endl;
//std::cout<<"this is gutman compute:\n "<<n_<<", "<< dis_plane<< endl;
	PointT Pi;
	float si = 0;
	float svd_si = 0;
	for (int i = 0; i< sigema.size();i++){
		Pi = sigema.points[i];
		pi << Pi.x, Pi.y, Pi.z;
//		float a =  (pi.transpose() )* n_ - dis_plane;
		float b =  (pi.transpose() )* n_svd - dis_svd;
//		si += sq(a);
		svd_si += sq(b);
	}
  //      RMS = sqrt(si /= sigema.size());
dis_plane = dis_svd;  

n.x = n_svd(0,0);
n.y = n_svd(1,0);
n.z = n_svd(2,0);


float      RMS = sqrt(svd_si /= sigema.size());
//std::cout<<"RMS =  "<<RMS <<" rms_SVD = ::" <<RMS_svd <<std::endl;
//	double RMS_time = pcl::getTime();
	//std::cout<<"RMS time .."<<rms_t - n_d_over<<" RMS___t "<<RMS_time - rms_t<< std::endl;
	if( RMS <= theta_seed){//TODO
//plane_cloud.clear();//TODO
		return true;
	}
	else return false;
}

float region_grow::det(PointT &a, PointT &b, PointT &c){
	float derta = a.x*b.y*c.z - a.x*b.z*c.y
		- a.y*b.x*c.z + a.y*b.z*c.x
		+ a.z*b.x*c.y - a.z*b.y*c.x;//TODO fuck Cramer, fuck 
	return derta;
}

void region_grow::add_into_patch(split_line::iterator &a){
	if(patch.size() == 0)
		std::cout<<"ADD	INTO patch error "<<std::endl;
	patch.push_back(get_data(a));
	patch_no++;
	planes.back().size++;
//	plane_cloud.points.push_back(a->end_point);
}

void region_grow::make_patch (split_line::iterator &a, split_line::iterator &b, split_line::iterator &c){
	if(patch.size()!=0){
		patch_map.push_back(patch); 
		patch.clear();
		planes.back().lines = patch_map.back();
//		std::cout<<" RMS... "<<RMS<<std::endl;
		plane_cloud.clear();
		plane_cloud.points.push_back(a->end_point);
		plane_cloud.points.push_back(a->end_point_e);
		plane_cloud.points.push_back(b->end_point);
		plane_cloud.points.push_back(b->end_point_e);
		plane_cloud.points.push_back(c->end_point);
		plane_cloud.points.push_back(c->end_point_e);
	}
	Plane new_plane;
	new_plane.RMS = RMS;
	new_plane.n  = n;
	new_plane.d = dis_plane;	
	new_plane.size = 3;
	planes.push_back(new_plane);
	
	patchmap_no++;
	patch.push_back(get_data(a));
	patch.push_back(get_data(b));
	patch.push_back(get_data(c));
}

PointT region_grow::minus(PointT &a, PointT &b){
	PointT c = a;
	c.x -= b.x;
	c.y -= b.y;
	c.z -= b.z;
	return c;
}

float  region_grow::mar_mul_R1(PointT &n_3, PointT &b_3){//return vec3f dot vec3f
	float op = n_3.x* b_3.x + n_3.y * b_3.y + n_3.z * b_3.z;
	return op;
}

/*PointCloudT region_grow::get_normals(){
if (normals.size()>0)
return normals;
}*/


bool region_grow::recom_plane(PointCloudT &sigema){
int siz  = sigema.size();	
if (siz < 6)
return false;
MatrixXf SVD_A(siz,4);
for(int i = 0; i< siz;  i++){
SVD_A (i,0) =  sigema.points[i].x;
SVD_A (i,1) =  sigema.points[i].y;
SVD_A (i,2) =  sigema.points[i].z;
SVD_A (i,3) =  -1.0;
}
JacobiSVD<MatrixXf> svd(SVD_A, ComputeThinU | ComputeThinV);

	Matrix<float, 3,1> n_, pi, n_svd, n_svd_norm;
	Matrix3f M;
//	n_<< n.x, n.y, n.z;
//	m_ = m;

Matrix <float, 4, 4> m_v = svd.matrixV();
n_svd<< m_v(0,3),m_v(1,3), m_v(2,3);
float dis_svd =  m_v(3, 3);
dis_svd /= n_svd.norm();
n_svd.normalize();
if (dis_svd < 0){
dis_svd *= -1;
n_svd *= -1;
}
	PointT Pi;
	float si = 0;
	float svd_si = 0;
	for (int i = 0; i< sigema.size();i++){
		Pi = sigema.points[i];
		pi << Pi.x, Pi.y, Pi.z;
//		float a =  (pi.transpose() )* n_ - dis_plane;
		float b =  (pi.transpose() )* n_svd - dis_svd;
//		si += sq(a);
		svd_si += sq(b);
	}
  //      RMS = sqrt(si /= sigema.size());


float      RMS = sqrt(svd_si /= sigema.size());
//std::cout<<"RMS =  "<<RMS <<std::endl;
//	double RMS_time = pcl::getTime();
	//std::cout<<"RMS time .."<<rms_t - n_d_over<<" RMS___t "<<RMS_time - rms_t<< std::endl;
	if( RMS <= theta_grow){//TODO
//plane_cloud.clear();//TODO
//	plane_cloud = sigema;
dis_plane = dis_svd;  

n.x = n_svd(0,0);
n.y = n_svd(1,0);
n.z = n_svd(2,0);
//std::cout<<" det A : "<<derta <<"  n is :" << n<< "  d is :" <<dis_plane<<std::endl;
		return true;
	}
	else{
	plane_cloud.points.pop_back();
	plane_cloud.points.pop_back();
 return false;}
}

split_image region_grow::merge_plane(){
	Planes::iterator itr_1 = planes.begin();
	Planes::iterator itr_2 = itr_1+1;
	for(;itr_1 != planes.end();){
		split_line new_patch = itr_1->lines;
		for (; itr_2 != planes.end();){
			PointT n1, n2;
			n1 = itr_1->n;
			n2 = itr_2->n;
			float d1,d2;
			d1 = itr_1->d;
			d2 = itr_2->d;
			//if((mar_mul_R1(n1,n2)> 0.8*sqrt((sq(n1.x)+sq(n1.y)+sq(n1.z))*(sq(n2.x)+sq(n2.y)+sq(n2.z)) ) )&&std::abs(d1-d2 <= 0.5 ) ){
			if((std::abs(d1-d2) <= 0.2 )&&(mar_mul_R1(n1,n2)>0.85)) {
				split_line a  =  itr_2->lines;
				comtru_no++;
				split_line::iterator a_itr = a.begin();
//			std::cout<<" d1 " << d1<< " d2 " <<d2 <<" n1 " <<n1<<" n2 :" << n2<<std::endl;
				for (; a_itr != a.end();){
					split_point pt =region_grow::get_data(a_itr);
					new_patch.push_back(pt);
					++a_itr;
				}
				//		i++;
				if(itr_2+1 != planes.end())
				planes.erase(itr_2++);	
				//patch_map.erase(patch_map.at(j+i));
				else ++itr_2;
			}
			else ++itr_2;
		}

		merge_map.push_back(new_patch); 
		++itr_1;
	}
	std::cout<<"comp no "<<comtru_no<< " merge result no "<< merge_map.size()<<std::endl;
	return merge_map;
}
