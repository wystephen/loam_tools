#include "laser_geometry/laser_geometry.h"
//#include "laser_geometry.cpp"
#include "tf/tf.h"


#include <pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include <fstream>


ros::Subscriber sub;
ros::Publisher pub;

//serial::Serial::Serial("/dev/ttyUSB0",115200,10,eightbits,parity_none,stopbits_one,flowcontrol_none);


double w=0;
int last_sum;
double last_endtime;

//for debug
std::fstream fout;

double time_before,time_now;
double sum_before,sum_now;
sensor_msgs::PointCloud2 pointcloud;
bool firsttime(true);

void lCallback(const sensor_msgs::PointCloud2::ConstPtr &tfmsg)
{
//    if(firsttime)
//    {
//        pointcloud = *tfmsg;
//    }
//    //pointcloud.datatfmsg->data;
//   pointcloud.header = tfmsg->header;
//   pointcloud.height =tfmsg->height;
//   pointcloud.width += tfmsg->width;
    pub.publish(pointcloud);

}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"sum_pointcloud");
    ros::NodeHandle n;
    ROS_INFO("Now,start the node succeful!");

//   sub = n.subscribe("/sync_scan_cloud_filtered",1,lcallback);
//pub=n.advertise<sensor_msgs::PointCloud2>("sum_of_pointcloud",1);

    ROS_INFO("step2111");





    ros::spin();
    return 0;
}
