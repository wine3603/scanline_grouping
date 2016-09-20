/*******************************************************
* Copyright (c) 2016 1 4 the Univ. of Tokyo || YNL
*
* @file rdpl.cpp
* @ brief relization of Ramen Douglas Peucker Line Simplification algorithm
* @ author Tianwei Zhang
**********************************************************/
#ifndef RDPL_H
#define RDPL_H
#include <ros/ros.h>
#include <iostream>
#include <math.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

#include <region_grow.h>

using namespace std;
using namespace Eigen;
using namespace region_growing;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const float bad_point = std::numeric_limits<float>::quiet_NaN();
class rdpl {
// make a stack data structure to store RDP iterative elements
public:
//template <typename PointT>  
float PointLineDistance(PointT&  point, PointT& start, PointT& end) {
	//if (start == end) {
		//return Vector2.Distance(point, start);
	//	return 0;	
	//	}
	
	float n = fabs((end.x - start.x) * (start.y - point.y) - (start.x - point.x) * (end.y - start.y));
	float d = sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));
	
	return n / d;
	}

//struct StackElement
//	{
//	  PointT point;
//	  size_t index;
//	};
//	std::vector< StackElement> rdp_stack;
	split_line rdp_stack;
	split_point StackElement;
//private:
// rdp function , with a given threshod epsilon, return 0 or 1 (bigger dis)

int rdp_implimentation(       PointCloudT& input,
                              split_line& output,//PointCloudT& output,
                                  double e )
{
    output.resize( 0 );

    if ( input.empty( ) )
    {
        return 0;
    }

    // Clean up
    rdp_stack.resize( 0 );
//std::cout<<"input.."<<input.size()<<"..output"<<output.size()<<"...epsilon: "<<e<<std::endl;
/*   if (rdp_stack.reserve( input.size( ) ) <= 0
        || output.reserve( input.size( ) ) <= 0 )
    {
        // Failed to allocate memory
        return -1;
    }
*/
    PointT anchor = input.points[0];
    size_t index_of_anchor = 0;
    size_t length_of_anchor = 0;
    PointT floater = input.points[input.size()];
    size_t index_of_floater = input.size( ) - 1;
    size_t length_of_floater = 0;

    // Add the first point in the poly to the result
	split_point temp_point_n;
	temp_point_n.end_point = anchor;
	temp_point_n.index = 0;//TODO
	temp_point_n.length = 0;
    output.push_back(temp_point_n );

    split_point stack_element =
    {
        floater,
        index_of_floater,
	length_of_floater
    };

    rdp_stack.push_back( stack_element );

 while ( !rdp_stack.empty( ) )
    {
        double max_squared_distance = 0.0;
        PointT farthest = anchor;
        size_t index_of_farthest = index_of_anchor;

        // Find point furthest from line defined by anchor and floater
        // function depends on your projection/dimension
       // DistanceHelper distance_helper( anchor, floater );

        for ( size_t i = index_of_anchor + 1; i < index_of_floater; ++i )
        {

            const double squared_distance = PointLineDistance (input.points[i], anchor, floater);// distance_helper.squared_distance_to( input[ i ] );
            if ( squared_distance > max_squared_distance )
            {
                max_squared_distance = squared_distance;
                farthest = input.points[i];
                index_of_farthest = i;
            }
        }

/////adaptive e	
	float range = farthest.intensity;
	if( 0.1  <= range && range  < 1.0)
	e = 0.05;
	else if ( range < 5.0)
	e = 0.05;
	else if (range< 10)
	e = 0.05;
	else e = 0.1;
 
        // Furthest point nearer than tolerance?
        if ( max_squared_distance <= e )
        {
	output.back().end_point_e = floater;
	output.back().index_e = index_of_floater;
            output.push_back( rdp_stack.back( ) );
            rdp_stack.pop_back( );
            anchor = floater;
            index_of_anchor = index_of_floater;
            if ( !rdp_stack.empty( ) )
            {
                floater = rdp_stack.back( ).end_point;
                index_of_floater = rdp_stack.back( ).index;
            }
        }
        else
        {
            floater = farthest;
            index_of_floater = index_of_farthest;
            stack_element.end_point = floater;
            stack_element.index = index_of_floater;
            rdp_stack.push_back( stack_element );
        }
    }
//////least square opt of end points 2016 04 26
/*
 split_line::iterator itr_r = output.begin();
 for (;itr_r != output.end();){

	 int n = itr_r->index_e - itr_r->index;
float RMS = 0;
	 if ( n >= 5){
//		std::cout<< "n is "<<n;
		 MatrixXf m(n, 3);
		 for(int i = 0; i < n; i++){
			 m(i,0)=input.points[itr_r->index + i].x;
			 m(i,1)=input.points[itr_r->index + i].y;
			 m(i,2)= 1.0;
		 }
		 JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);
		 Matrix <float, 3, 3> m_v = svd.matrixV();
		 Matrix <float, 3, 1> L;//L is the line formular ax + by +c = 0
		 L <<m_v(0,2), m_v(1,2), m_v(2,2);
		// for(int i = 0; i < n; i++){
		//	Matrix <float, 1, 3> m_p;
		//	PointT pt = input.points[itr_r->index+i];
		//	
		//	m_p << pt.x, pt.y, -1;
		//	RMS += m_p * L; 
		// }
		PointT pt = itr_r->end_point;
		float dis = fabs(pt.x*L(0,0) + pt.y*L(1,0) +L(2,0))/sqrt( L(0,0)*L(0,0) + L(1,0)*L(1,0));
	Matrix <float, 2,2> A;// qiu chui zu
	A(0,0) = L(0,0);
	A(0,1) = L(1,0);
	A(1,0) = L(1,0);
	A(1,1) = -L(0,0);
	Vector2f  b, p;
	float x_0 = pt.x;
	float y_0 = pt.y;
	b<< -L(2,0), (L(1,0)*x_0 - L(0,0)*y_0);
 ColPivHouseholderQR<Matrix2f>  dec(A);
 p = dec.solve(b); 
itr_r->end_point.x = p(0,0);
itr_r->end_point.y = p(1,0);
	 }
++itr_r;
}
 */   // Success
    return 0;
};
};
#endif
