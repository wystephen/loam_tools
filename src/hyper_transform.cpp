//
// Created by lixin on 16-3-7.
//

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/timer.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include  <vector>
#include <queue>
#include <deque>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>


#include "laser_geometry/laser_geometry.h"


#define PARAMENTER_OPTIMATION

//ros::Subscriber sub_para;
ros::Subscriber sub_laser;

ros::Publisher pub;

//Read serial data struct
boost::asio::io_service io;
boost::asio::serial_port sp(io,"/dev/ttyUSB0");


//mutex and lock
typedef boost::shared_lock<boost::shared_mutex> readLock;
typedef boost::unique_lock<boost::shared_mutex> writeLock;

boost::shared_mutex rwmutex;

//angle data
double global_angle;
char read_buf[10];

struct angle_with_time{
    double time;
    double angle;
};

angle_with_time global_angle_with_time;
std::deque<angle_with_time> angle_queue;

//hyper_para struct
struct HyperParameter{
    double avg_v;
    double offset_r;
    double error_theta_z;

}global_para;

//device stop function
bool rotation_stop(){
    //stop  AA 55 0A 02 0C
    boost::asio::write(sp,boost::asio::buffer("\xAA",1));
    boost::asio::write(sp,boost::asio::buffer("\x55",1));
    boost::asio::write(sp,boost::asio::buffer("\xFA",1));
    boost::asio::write(sp,boost::asio::buffer("\x02",1));
    boost::asio::write(sp,boost::asio::buffer("\xFc",1));
    return true;

}

//device start function
bool rotation_start()
{
    boost::asio::write(sp,boost::asio::buffer("\xAA",1));
    boost::asio::write(sp,boost::asio::buffer("\x55",1));
    boost::asio::write(sp,boost::asio::buffer("\xFA",1));
    boost::asio::write(sp,boost::asio::buffer("\x01",1));
    boost::asio::write(sp,boost::asio::buffer("\xFB",1));
    return true;
}

//return angle only,use mutex.
double readonly(){
    readLock rdlock(rwmutex);

    return global_angle;
}
//read only function return angle with time struct data
angle_with_time readOnly_time(double laser_time){
    readLock rdlock(rwmutex);
    angle_with_time tmp_a_t;
    for(int i(angle_queue.size()-1);i--;i >= 1)
    {
        if(fabs(angle_queue[i].time-laser_time)<0.015 && fabs(angle_queue[i-1].time-laser_time) < 0.015)
        {
            //direct use it
            return angle_queue[i];
            //liner interpolartion
            tmp_a_t.time = laser_time;
            /********************************************************* /
            if(angle_queue[i].angle<angle_queue[i+1].angle)
            {
                tmp_a_t.angle = angle_queue[i-1].angle +
                        ((angle_queue[i].angle+((double) 7200.0/20.0/180.0 * M_PI))-angle_queue[i-1].angle)
                        *(laser_time - angle_queue[i-1].time)/ (angle_queue[i].time - angle_queue[i-1].time);
                if(tmp_a_t.angle>(7200.0/20.0/180.0)*M_PI)
                {
                    tmp_a_t.angle -= (7200.0/20.0/180.0*M_PI);
                }
            }else{
                tmp_a_t.angle = angle_queue[i-1].angle +
                                ((angle_queue[i].angle)-angle_queue[i-1].angle)
                                *(laser_time-angle_queue[i-1].time)/(angle_queue[i].time - angle_queue[i-1].time);

            }
             /******************************************/
            // can update global parameter avg_v
            if(angle_queue[i+1].angle<angle_queue[i-3].angle)
            {
                global_para.avg_v = (double)(angle_queue[i+1].angle + (7200.0/20.0/180.0*M_PI) - angle_queue[i-3].angle)
                        /(angle_queue[i+1].time-angle_queue[i-3].time)/M_PI * 20.0 * 180.0;
            }else{
                global_para.avg_v = (double)(angle_queue[i+1].angle - angle_queue[i-3].angle)
                                    /(angle_queue[i+1].time-angle_queue[i-3].time)/M_PI * 20.0 * 180.0;
            }

            tmp_a_t.angle = angle_queue[i].angle + (laser_time - angle_queue[i].time) * global_para.avg_v;


            return tmp_a_t;
        }
    }

    return global_angle_with_time;
}


laser_geometry::LaserProjection p;
void lCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    angle_with_time angle_t;
    angle_t = readOnly_time(scan_msg->header.stamp.toSec());

    sensor_msgs::PointCloud2 pointcloud_tmp;

    double scan_time,scan_diff,point_scan_diff,point_cloud_time;

    scan_time= scan_msg->header.stamp.toSec();

    p.projectLaser(*scan_msg,pointcloud_tmp,-1.0,3);//1.0 - 2.0  or -1.0

    point_cloud_time = pointcloud_tmp.header.stamp.toSec();

    point_scan_diff=point_cloud_time-scan_time;
    pointcloud_tmp.header.frame_id="/camera";



    double angle(0.0);


    /************************************************************************************************ /
    //time diff output code

    //std::cout << "angle:"<<angle_t.angle<<std::endl;
    std::cout <<"[sys_s_diff:sys_l_diff:l_s_diff]:"<<
            ros::Time::now().toSec()-angle_t.time<<":"<<
            ros::Time::now().toSec()-scan_time<<":"<<
            angle_t.time-scan_msg->header.stamp.toSec()<<":"<<
            global_para.avg_v<<":"<<std::endl;
    /*********************************************************************************************************************************************************/

    /***********************************************************************************************************************/
    std::cout <<"angle_t.angle:"<<angle_t.angle<<std::endl;

    double avg_v((double) global_para.avg_v / 20.0 / 180.0 * M_PI);
    angle = angle_t.angle - (angle_t.time-scan_msg->header.stamp.toSec()) * avg_v;
while(angle < 0.0 || angle > (7200.0/20/180.0 *M_PI))
{
    if(angle<0.0){
        angle += (7200.0/20/180.0 * M_PI);
    }
    if(angle > (7200.0/20/180.0 * M_PI))
    {
        angle -= (7200.0/20/180.0 * M_PI);
    }
}


    double theta = angle;// radian
    double err_theta_z = global_para.error_theta_z;
    double offset_r = global_para.offset_r;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f t_transform = Eigen::Matrix4f::Identity();


    ///////////////////////////////////
    //copy from fast_transform
    t_transform(0,0) = cos(err_theta_z);
    t_transform(1,0)  = -sin(err_theta_z);
    t_transform(0,1) = sin(err_theta_z);
    t_transform(1,1) = cos(err_theta_z);
    t_transform(2,2) = 1.0;
    t_transform(3,3) = 1.0;


    int x_idx = pcl::getFieldIndex(pointcloud_tmp,"x");
    int y_idx = pcl::getFieldIndex(pointcloud_tmp,"y");
    int z_idx = pcl::getFieldIndex(pointcloud_tmp,"z");


    //check if distance is available
    int dist_idx = pcl::getFieldIndex(pointcloud_tmp,"distance");
    //xyz_offset
    Eigen::Array4i xyz_offset (pointcloud_tmp.fields[x_idx].offset,pointcloud_tmp.fields[y_idx].offset,pointcloud_tmp.fields[z_idx].offset,0);
    double ntheta;
    int static ok=0;


    transform(0,0)=1;
    transform(3,3) = 1;



    //=============================================================================================
    for (size_t i = 0;i<pointcloud_tmp.width * pointcloud_tmp.height;++i)
    {

        Eigen::Vector4f pt(*(float *)&pointcloud_tmp.data[xyz_offset[0]],*(float*)&pointcloud_tmp.data[xyz_offset[1]],*(float*)&pointcloud_tmp.data[xyz_offset[2]],1);
        Eigen::Vector4f pt_out;


        //ntheta = theta//+((0.025*3.14159265*(double)avg_v/180/20/8)+
        // ntheta = theta   +(((135+(double)(atan2(pt[1], pt[0])*180/3.14159265)))*(double)avg_v*0.025/20*3/4*3.1415926/180/270);
        //ntheta = theta+((0.025* M_PI *(double)avg_v/180/20/8)+(((135+(double)(atan2(pt[1], pt[0])*180/M_PI)))*(double)avg_v*0.025/20*3/4*3.1415926/180/270));
        ntheta = theta - (1-(M_PI+atan2(pt[1],pt[0]))/2/M_PI)*(avg_v*0.025/20.0/180*M_PI);
        //fout <<ntheta *180 /3.1415926<<std::endl;


        transform(1,1)=cos(ntheta);
        transform(1,2)=-sin(ntheta);
        transform(2,1)=sin(ntheta);
        transform(2,2)=cos(ntheta);
        transform(2,3)=cos(ntheta)* offset_r ;
        transform(1,3)=-sin(ntheta)*  offset_r ;


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
                pt_out = t_transform * pt;
                pt_out = transform * pt_out;//
                max_range_point = true;
            }
        }else{
            pt_out = t_transform * pt;
            pt_out = transform * pt_out;
        }

        if(max_range_point)
        {
            *(float*)(&pointcloud_tmp.data[distance_ptr_offset]) = pt_out[0];
            pt_out[0] = std::numeric_limits<float> ::quiet_NaN();
        }

        pt_out[2] = - pt_out[2];


        memcpy(&pointcloud_tmp.data[xyz_offset[2]],&pt_out[0], sizeof(float));
        memcpy(&(pointcloud_tmp.data[xyz_offset[0]]),&pt_out[1], sizeof(float));
        memcpy(&pointcloud_tmp.data[xyz_offset[1]],&pt_out[2], sizeof(float));

        xyz_offset += pointcloud_tmp.point_step;

    }


    ok++;
    pub.publish(pointcloud_tmp);

    if(!ros::ok()){
        rotation_stop();
    }

}


//use mutex
void writeOnly(double ang){
    writeLock wtlock(rwmutex);
    global_angle = ang;
}


void writeOnly_time(double angle,double time){
    writeLock wtlock(rwmutex);
    global_angle_with_time.angle=angle;
    global_angle_with_time.time = time;
    angle_queue.push_back(global_angle_with_time);
    if(angle_queue.size() >= 10)
        angle_queue.pop_front();
}



void serial_process()
{
    if(!ros::ok())
    {
        rotation_stop();
        return;//close when ros node end.
    }
    while(ros::ok()) {
        memset(read_buf, 0, 10);
        if (sp.is_open()) {
            boost::asio::read(sp, boost::asio::buffer(read_buf));
        }
        double the_time = ros::Time::now().toSec();
        //std::cout <<"read buf"<<std::endl;

        for (int i = 0; i < 10; i++) {
            //std::cout << "i:"<<i<<std::endl;
            //read_buf[i] = read_buf[i] & 0xff;

            //process the
            if ((read_buf[i] & 0xff) == 0x55) {
                int a = 0xff & read_buf[i + 1];
                int b = 0xff & read_buf[i + 2];
                int c = 0xff & read_buf[i + 3];
                if ((b > -1) && (c = ((a + b) % 256)))
                {
                    int sum = a * 256 + b;
                    if (sum < 7201) {
                        if(i<5)
                            the_time -=0.01;
                       // boost::mutex::scoped_lock(io_mutex);
                        //std::cout << "sum:"<<sum << std::endl;io_lock.lock();

                        //global_angle = (double)sum / 20 / 180 * M_PI;
                        //writeOnly((double) sum /20.0/180 * M_PI);
                        writeOnly_time((double) sum /20.0/180 * M_PI,the_time);

                        //std::cout <<"global angle:"<<global_angle<<std::endl;
                    }
                }
            }
        }
    }
}

void seril_fast()
{
    int a(0);
    int b(0);
    int c(0);
    int sum(0);
    double the_time;

    while(ros::ok())
    {
        memset(read_buf,0,10);

        boost::asio::read(sp,boost::asio::buffer(read_buf,1));
        the_time = ros::Time::now().toSec();

        if((read_buf[0] & 0xff) == 0xAA)
        {

            boost::asio::read(sp,boost::asio::buffer(read_buf,3));
            a = read_buf[1] & 0xff;
            b = read_buf[2] & 0xff;
            sum = a * 256 + b;

            if(sum<7201) writeOnly_time((double) sum /20.0/180 * M_PI,the_time);


        }else if((read_buf[0] & 0xff) == 0x55)
        {
            boost::asio::read(sp,boost::asio::buffer(read_buf,2));
            a = read_buf[0] & 0xff;
            b = read_buf[1] & 0xff;
            sum = a * 256 + b;
            if(sum<7201)
                writeOnly_time((double) sum /20.0/180 * M_PI,the_time);


        }


    }
    rotation_stop();
    return;

}

int main(int argc,char **argv)
{
    ROS_INFO("Begin Hyper transform");

    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control());
    sp.set_option(boost::asio::serial_port::parity());
    sp.set_option(boost::asio::serial_port::stop_bits());
    sp.set_option(boost::asio::serial_port::character_size(8));

    ros::init(argc,argv,"hyper_transform");

    ros::NodeHandle node;

    pub  = node.advertise<sensor_msgs::PointCloud2>("sync_scan_cloud_filtered",1);
    sub_laser = node.subscribe("/first",1,lCallback);


    global_para.avg_v = 7200;
    global_para.offset_r = 0.065000;
    global_para.error_theta_z = 0.304000 * 0.0174;

    rotation_start();

    //ros::spin();

    //boost::thread serial_process_thread(&serial_process);
    boost::thread serial_process_thread(&seril_fast);
    serial_process_thread.detach();

    sleep(2);

    //serial_process_thread.join();
    //serial_process_thread.detach();

    while(ros::ok()){
        ros::spinOnce();
        serial_process_thread.join();
        if(!ros::ok()) rotation_stop();
    }

    return 0;
    //sub_para = node.subscribe("hyper_para",1,paraCallback);

}