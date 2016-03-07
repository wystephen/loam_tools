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
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

std::fstream fout;
ros::Subscriber sub;

#define SUM_TIME 200
#define SAVE_TIME 30
#define SET_V 600

#define IS_WRITE false


//pcl::PointCloud<pcl::PointXYZ>::Ptr sum_ptr(new pcl::PointCloud<pcl::PointXYZ>());
int sum_times(0);
pcl::PointCloud<pcl::PointXYZ> sum_pt;
int save_number(SAVE_TIME);

double evaluation(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
pcl::PointCloud<pcl::PointXYZ>::Ptr target){
    pcl::KdTreeFLANN<pcl::PointXYZ> src_kdtree;
    src_kdtree.setInputCloud(source);

    double sum(0.0);

    for(int index(0);index < target->size();index++){
        std::vector<int> near_idx_list;
        std::vector<float> near_sqdis_list;

        if(src_kdtree.nearestKSearch(target->at(index),1,near_idx_list,near_sqdis_list) > 0){
            sum += near_sqdis_list[0];

        }

    }
    return sum / target->size();



}

void readCallback(const sensor_msgs::PointCloud2::ConstPtr& scan_msg)
{
    if(sum_times>SUM_TIME||sum_times == 0)
    {
        //sum_ptr = new pcl::PointCloud<pcl::PointXYZ>();
        //sum_pt=new pcl::PointCloud<pcl::PointXYZ>
        //pcl::PointCloud<pcl::PointXYZ> sum_pt;//(new pcl::PointCloud<pcl::PointXYZ>);
        if(save_number>0&&sum_pt.size()>10){
            //std::string tmp_file;

            if(IS_WRITE){
                std::stringstream ss;
                ss<<"pc_"<<SAVE_TIME-save_number<<"<"<<SUM_TIME<<"_"<<SET_V<<".pcd";
                pcl::io::savePCDFile(ss.str(),sum_pt);
                std::cout << "succeful write file!"<<std::endl;
                save_number--;
            }

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


    point_cloud_time = pointcloud_tmp.header.stamp.toSec();

    pointcloud_tmp.header.frame_id="/camera";

    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudptr(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg(*scan_msg,*laserCloudptr);
    sum_pt += *laserCloudptr;
    sum_times++;
    std::cout << sum_times<<std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sum_pointcloud");
    ros::NodeHandle n;
    ROS_INFO("Start get pointcloud.");

    //fout.open("/home/lixin/pointcloudxyz");

    sub = n.subscribe("sync_scan_cloud_filtered",1,readCallback);
    //double *sebig = new double[2^27];




    ros::spin();

    return 0;
}

