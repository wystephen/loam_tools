#include "laser_geometry/laser_geometry.h"
#include "tf/tf.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/timer.hpp>

#include <pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include <fstream>

#include <tf/transform_listener.h>

ros::Subscriber sub;
ros::Publisher pub;


double w=0;
int last_sum;
double last_endtime;

//for debug
std::fstream fout;
std::fstream foutlaser;

double time_before,time_now;
double yaw_before,sum_now;

static double scan_time_old=0;




void lCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{

    // double gettime = ros::Time::now().toSec();

    //ROS_INFO("1");
    laser_geometry::LaserProjection p;

    sensor_msgs::PointCloud2 pointcloud_tmp;

    double scan_time,scan_diff,point_scan_diff,point_cloud_time;

    scan_time= scan_msg->header.stamp.toSec();
    scan_diff=scan_time-scan_time_old;
    scan_time_old=scan_time;
    p.projectLaser(*scan_msg,pointcloud_tmp,-1.0,3);//1.0 - 2.0  or -1.0

    point_cloud_time = pointcloud_tmp.header.stamp.toSec();


    point_scan_diff=point_cloud_time-scan_time;
    pointcloud_tmp.header.frame_id="/camera_2";

    Eigen::Matrix4f Tm= Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tm1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tm2 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tm3 = Eigen::Matrix4f::Identity();


    static tf::TransformListener listener;
    static tf::StampedTransform tf_transform;
    try{
        listener.lookupTransform("/laser","/camerat",ros::Time(0),tf_transform);
    }catch(tf::TransformException &ex){
        ros::Duration(1.0).sleep();
        return;
    }
    tf::Quaternion q =tf_transform.getRotation();

    //double x = tf_transform.getRotation().getX();


    //double y = tf_transform.getRotation().getY();
    //double z = tf_transform.getRotation().getZ();
    //double w = tf_transform.getRotation().getW();
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double delta_time = ros::Time::now().toSec() - (tf_transform.stamp_.toSec());

    yaw = yaw + delta_time * 1.3 * 1/abs(yaw/3.1415926) *((yaw - yaw_before) / (ros::Time::now().toSec() - time_before));

    double theta = -(yaw-3.1415926/2)-3.1415926/2;

    yaw_before = yaw;
    time_before = ros::Time::now().toSec();
    //ROS_INFO("%f",theta);
    //ROS_INFO("w is:%f",w);

    Tm(0,0)=1;
    Tm(1,1)=cos(theta);
    Tm(1,2)=-sin(theta);
    Tm(2,1)=sin(theta);
    Tm(2,2)=cos(theta);


   // Tm(2,3)=cos(theta) * 0.04;// * 50;//axis-z
    //Tm(1,3)=-sin(theta) * 0.04 ;//* 50;//axis-y
    Tm(3,3)=1;

    //z -90
    Tm3(0,1)=1;
    Tm3(1,0)=-1;
    Tm3(2,2)=1;

    //x -90
    Tm2(0,0)=1;
    Tm2(1,2)=1;
    Tm2(2,1)=-1;
    // ROS_INFO("4");
    //z +180
    Tm1(0,0)=-1;
    Tm1(1,1)=-1;
    Tm1(2,2)=1;


    std::cout<< "pointcloud data:"<<pointcloud_tmp.data[20]<<"   size:"<<pointcloud_tmp.data.size()<<std::endl;

    std::cout <<"list"<<pcl::getFieldsList(pointcloud_tmp)<<std::endl;
    //get the really y z of the point
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///
    ///
    ///
    ///
    ///
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //   //out
    sensor_msgs::PointCloud2 out;
    //   //define a transfomr (may same as Tm)
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform = Tm;
    //   transform(0,0)=1;
    //   transform(1,1)=cos(theta);
    //   transform(1,2)=-sin(theta);
    //   transform(2,1)=sin(theta);
    //   transform(2,2)=cos(theta);
    //   transform(2,3)=cos(theta) * 0.035;
    //   transform(1,3)=-sin(theta) * 0.035;
    //   transform(3,3) = 1;
    // Get X-Y-Z indices;
    int x_idx = pcl::getFieldIndex(pointcloud_tmp,"x");
    int y_idx = pcl::getFieldIndex(pointcloud_tmp,"y");
    int z_idx = pcl::getFieldIndex(pointcloud_tmp,"z");


    //check if distance is available
    int dist_idx = pcl::getFieldIndex(pointcloud_tmp,"distance");
    //xyz_offset
    Eigen::Array4i xyz_offset (pointcloud_tmp.fields[x_idx].offset,pointcloud_tmp.fields[y_idx].offset,pointcloud_tmp.fields[z_idx].offset,0);
    double ntheta;

    int static ok=0;
    for (size_t i = 0;i<pointcloud_tmp.width * pointcloud_tmp.height;++i)
    {

        Eigen::Vector4f pt(*(float *)&pointcloud_tmp.data[xyz_offset[0]],*(float*)&pointcloud_tmp.data[xyz_offset[1]],*(float*)&pointcloud_tmp.data[xyz_offset[2]],1);
        Eigen::Vector4f pt_out;




        ntheta = theta;//-((90-atan(pt[0] / pt[1]))/270*0.025*avg_v);//-;
        ntheta = 0;
        transform(0,0)=1;
        transform(1,1)=cos(ntheta);
        transform(1,2)=-sin(ntheta);
        transform(2,1)=sin(ntheta);
        transform(2,2)=cos(ntheta);
        transform(2,3)=cos(ntheta);// * 0.033;
        transform(1,3)=-sin(theta);// * 0.033;
        transform(3,3) = 1;


        if(ok<10)
            foutlaser<<180*theta<<","<< pt[0]<<","<< pt[1] <<","<<180*ntheta<<std::endl;




        bool max_range_point = false;
        int distance_ptr_offset = i * pointcloud_tmp.point_step + pointcloud_tmp.fields[dist_idx].offset;
        float * distance_ptr = (dist_idx<0?NULL:(float*)(&pointcloud_tmp.data[distance_ptr_offset]));
        if(!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2]))
        {
            if (distance_ptr == NULL || !std::isfinite(*distance_ptr))   //Invalid point
            {
                pt_out = pt;
            }else{//max range point
                pt[0] = *distance_ptr; //Replace x with the x value saved in distance
                pt_out = transform * pt;//
                max_range_point = true;
            }
        }else{
            pt_out = transform * pt;
        }

        if(max_range_point)
        {
            //Save x. value in distance again
            *(float*)(&pointcloud_tmp.data[distance_ptr_offset]) = pt_out[0];
            pt_out[0] = std::numeric_limits<float> ::quiet_NaN();
        }

        memcpy(&pointcloud_tmp.data[xyz_offset[0]],&pt_out[0], sizeof(float));
        memcpy(&pointcloud_tmp.data[xyz_offset[1]],&pt_out[1], sizeof(float));
        memcpy(&pointcloud_tmp.data[xyz_offset[2]],&pt_out[2], sizeof(float));

        xyz_offset += pointcloud_tmp.point_step;


    }

    ok++;

    //check if the viewpoint infomation is persent
    // int vp_idx = pcl::getFieldIndex(pointcloud_tmp,"vp_x");
    // if()


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///
    ///

    // pcl_ros::transformPointCloud(Tm,pointcloud_tmp,pointcloud_tmp);


    pcl_ros::transformPointCloud(Tm3,pointcloud_tmp,pointcloud_tmp);
    //pcl_ros::transformPointCloud(Tm3,pointcloud_tmp,pointcloud_tmp);

    //pcl_ros::transformPointCloud(Tm2,pointcloud_tmp,pointcloud_tmp);
    pcl_ros::transformPointCloud(Tm2,pointcloud_tmp,pointcloud_tmp);


    //  ROS_INFO("5");

    pub.publish(pointcloud_tmp);
    //fout<<sum<<","<<times<<","<<endtime;

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"laser_transfor_new");
    ros::NodeHandle n;
    ROS_INFO("Now,start the node succeful!");




    pub=n.advertise<sensor_msgs::PointCloud2>("sync_scan_cloud_filtered",1);
    sub=n.subscribe("first",1,lCallback);


    ros::spin();

    return 0;
}



