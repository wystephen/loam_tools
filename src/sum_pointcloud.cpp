#include "laser_geometry/laser_geometry.h"
#include "tf/tf.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <boost/asio.hpp>
#include <boost/timer.hpp>
#include <boost/bind.hpp>

#include <pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include <fstream>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

std::fstream fout;
ros::Subscriber sub;


//pcl::PointCloud<pcl::PointXYZ>::Ptr sum_ptr(new pcl::PointCloud<pcl::PointXYZ>());
int sum_times(0);
pcl::PointCloud<pcl::PointXYZ> sum_pt;
int save_number(30);
void readCallback(const sensor_msgs::PointCloud2::ConstPtr& scan_msg)
{
    if(sum_times>80||sum_times == 0)
    {
        //sum_ptr = new pcl::PointCloud<pcl::PointXYZ>();
        //sum_pt=new pcl::PointCloud<pcl::PointXYZ>
        //pcl::PointCloud<pcl::PointXYZ> sum_pt;//(new pcl::PointCloud<pcl::PointXYZ>);
        if(save_number>0&&sum_pt.size()>10){
            //std::string tmp_file;
            std::stringstream ss;
            ss<<"pcl_file"<<30-save_number<<".pcd";


            pcl::io::savePCDFile(ss.str(),sum_pt);
            std::cout << "succeful write file!"<<std::endl;
            save_number--;
        }
        sum_pt.clear();
        sum_times= 1;
    }
    std::cout << sum_pt.size()<<std::endl;

    // double gettime = ros::Time::now().toSec();

    ROS_INFO("1");
    //laser_geometry::LaserProjection p;

    sensor_msgs::PointCloud2 pointcloud_tmp;
    pointcloud_tmp = *scan_msg;


    double scan_time,scan_diff,point_scan_diff,point_cloud_time;

    scan_time= scan_msg->header.stamp.toSec();
    //scan_diff=scan_time-scan_time_old;
    //scan_time_old=scan_time;
    //p.projectLaser(*scan_msg,pointcloud_tmp,-1.0,3);//1.0 - 2.0  or -1.0

    point_cloud_time = pointcloud_tmp.header.stamp.toSec();

    //std::cout << "scantime is :;:::"<<scan_time <<std::endl;
    //std::cout << "pointclou is :::::"<< point_cloud_time <<std::endl;
    //point_scan_diff=point_cloud_time-scan_time;
    pointcloud_tmp.header.frame_id="/camera";



    int x_idx = pcl::getFieldIndex(pointcloud_tmp,"x");
    int y_idx = pcl::getFieldIndex(pointcloud_tmp,"y");
    int z_idx = pcl::getFieldIndex(pointcloud_tmp,"z");

    int dist_idx = pcl::getFieldIndex(pointcloud_tmp,"distance");
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudptr(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg(*scan_msg,*laserCloudptr);
    sum_pt += *laserCloudptr;
    sum_times++;
    std::cout << sum_times<<std::endl;



    Eigen::Array4i xyz_offset (pointcloud_tmp.fields[x_idx].offset,pointcloud_tmp.fields[y_idx].offset,pointcloud_tmp.fields[z_idx].offset,0);
    for (size_t i = 0;i<pointcloud_tmp.width * pointcloud_tmp.height;++i)
    {
        Eigen::Vector4f pt(*(float *)&pointcloud_tmp.data[xyz_offset[0]],*(float*)&pointcloud_tmp.data[xyz_offset[1]],*(float*)&pointcloud_tmp.data[xyz_offset[2]],1);
        int distance_ptr_offset = i * pointcloud_tmp.point_step + pointcloud_tmp.fields[dist_idx].offset;
        float * distance_ptr = (dist_idx<0?NULL:(float*)(&pointcloud_tmp.data[distance_ptr_offset]));

            if (distance_ptr == NULL || !std::isfinite(*distance_ptr))   //Invalid point
            {
                pt = pt;
            }else{//max range point
                pt[0] = *distance_ptr; //Replace x with the x value saved in distance
            }

        //std::cout<<i<<" : "<<pt[0]<<" "<<pt[1]<<" "<<pt[2]<<" "<<pt[3]<<std::endl;
        //std::cout<< pt[0]<<","<<pt[1]<<","<<pt[2]<<std::endl;
        xyz_offset += pointcloud_tmp.point_step;
    }




}

int main(int argc, char **argv){
    ros::init(argc, argv, "sum_pointcloud");
    ros::NodeHandle n;
    ROS_INFO("Start get pointcloud.");

    //fout.open("/home/lixin/pointcloudxyz");

    sub = n.subscribe("sync_scan_cloud_filtered",1,readCallback);
    //double *sebig = new double[2^27];




    ros::spin();
    //fout.close();

    return 0;
}

