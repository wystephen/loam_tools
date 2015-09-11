#include "laser_geometry/laser_geometry.h"
#include "tf/tf.h"

#include <pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include <fstream>

#include <tf/transform_listener.h>

#include<sys/types.h>
#include<fcntl.h>
#include<unistd.h>
#include<termio.h>
#include<stdlib.h>


int fd;

ros::Subscriber sub;
ros::Publisher pub;

bool thread_ok(true);

double w=0;
int last_sum;
double last_endtime;

double time_before,time_now;
double sum_before,sum_now;

static double scan_time_old=0;
double last_avg_v(600);


//for debug
std::fstream fout;
std::fstream foutlaser;



void lCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    laser_geometry::LaserProjection p;

    sensor_msgs::PointCloud2 pointcloud_tmp;

    double scan_time,scan_diff,point_scan_diff,point_cloud_time;

    scan_time= scan_msg->header.stamp.toSec();
    scan_diff=scan_time-scan_time_old;
    scan_time_old=scan_time;
    p.projectLaser(*scan_msg,pointcloud_tmp,-1.0,3);//1.0 - 2.0  or -1.0

    point_cloud_time = pointcloud_tmp.header.stamp.toSec();

    point_scan_diff=point_cloud_time-scan_time;
    pointcloud_tmp.header.frame_id="/camera";

    Eigen::Matrix4f Tm= Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tm1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tm2 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tm3 = Eigen::Matrix4f::Identity();


    bool isok(false);
    char bufread[1000];
    int a(0),b(0),c(0),sum;
    sum = 0;
    int times(0);
    double serial_time;

    while(isok==false)//ver 1.0--read
    {
        int data_long(0);
        while((data_long = read(fd,bufread,1000))<5)




        serial_time = ros::Time::now().toSec();


        for(int i=data_long-3;i>0;i--){
            if(bufread[i]=='\x55'){

                a = 0xff & bufread[i+1];
                b = 0xff & bufread[i+2];
                c = 0xff & bufread[i+3];

                if( (b > -1) && (c == ((a+b) % 256)) )
                {

                    sum = a * 256 + b;
                    std::cout << sum<<std::endl;

                    if(sum<7201 )
                    {
                        //if(i<1)
                            //serial_time-=0.01;
                        if( data_long-i>4)
                            serial_time-=0.01;
                        isok = true;
                        sum_now = sum;
                        if(i>4){
                            int aa,bb,cc;
                            aa = 0xff & bufread[i-4];
                            bb = 0xff & bufread[i-3];
                            cc = 0xff & bufread[i-2];
                            if( (bb>-1)&&(c==((a+b)%256)) ){

                            }
                        }
                        //std::cout <<"i:"<<i<<"  data_long:"<<data_long<<std::endl;
                        break;
                    }

                }
            }
        }

        times++;
        if(times > 1)
        {
            std::cout << "get sum false!"<<data_long<<std::endl;
            return;

        }
    }
    time_now = serial_time;

    double endtime = serial_time - scan_time;pointcloud_tmp.header.stamp.toSec();
    //std::cout << endtime <<std::endl;


    double avg_v;


    if(sum_now < sum_before)
        sum_now += 7200;
    avg_v = last_avg_v;//(((sum_before - sum_now)/(time_before - time_now)));
    //std::cout << (sum_before-sum_now)/(time_before-time_now)<<std::endl;
    sum = sum-endtime * avg_v ;
    if(sum < 0)
        sum +=7200;
    if(sum>7200) sum -= 7200;
    time_before = time_now;
    sum_before = sum;


    //w is the angle of lidar
    w =(sum /20 );//-180 ;
    last_endtime = endtime;
    last_sum = sum;
    float theta=3.1415926 * w / 180;
    //std::cout <<"last_avg_v:"<<last_avg_v<<"avg_v:"<<avg_v<<"endtime*avg_v:"<<endtime*avg_v<<"sum:"<<sum<<"w:"<<w<<std::endl;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////atan2////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///
    ///
    ///
    ///
    ///
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform = Tm;

    int x_idx = pcl::getFieldIndex(pointcloud_tmp,"x");
    int y_idx = pcl::getFieldIndex(pointcloud_tmp,"y");
    int z_idx = pcl::getFieldIndex(pointcloud_tmp,"z");


    //check if distance is available
    int dist_idx = pcl::getFieldIndex(pointcloud_tmp,"distance");
    //xyz_offset
    Eigen::Array4i xyz_offset (pointcloud_tmp.fields[x_idx].offset,pointcloud_tmp.fields[y_idx].offset,pointcloud_tmp.fields[z_idx].offset,0);
    double ntheta;
    int static ok=0;
    //std::cout <<"-------------------------------"<<std::endl;
    //std::ofstream fout;
    //std::ostringstream oss;
    //oss << "/home/lixin/save/"<<pointcloud_tmp.header.stamp<<".txt";
    //fout.open(oss.str().c_str());
    //fout <<avg_v<<std::endl;
    //fout <<sum<<std::endl;
    //fout << theta<<std::endl;
    for (size_t i = 0;i<pointcloud_tmp.width * pointcloud_tmp.height;++i)
    {

        Eigen::Vector4f pt(*(float *)&pointcloud_tmp.data[xyz_offset[0]],*(float*)&pointcloud_tmp.data[xyz_offset[1]],*(float*)&pointcloud_tmp.data[xyz_offset[2]],1);
        Eigen::Vector4f pt_out;


        ntheta = theta+(0.025/8*avg_v*3.1415926/180/20)*8/10+(((135+(double)(atan2(pt[1], pt[0])*180/3.14159265))/270)*avg_v*0.025/20*3/4*3.1415926/180)*8/10;
        //fout << ntheta<<","<<pt[0]<<","<<pt[1]<<","<<pt[2]<<std::endl;
        //std::cout<<((135+(double)(atan2(pt[1], pt[0])*180/3.14159265))/270*avg_v*0.025/20*3/4)<<std::endl;//<<"     " <<((135+(atan2(pt[1], pt[1])*180/3.14159265))/270*0.025*avg_v/20*3/4)/270*0.025<<std::endl;
                //ntheta = theta - avg_v*(0.025/8+(pointcloud_tmp.width-i)/pointcloud_tmp.width*0.025*3/4);
        //std::cout << atan2(pt[1], pt[0])*180/3.14159265<<std::endl;
        //std::cout<< theta<<"   "<<ntheta<<std::endl;
        //if(ntheta<0) ntheta+=6.2831852;

        //ntheta = theta;


        transform(0,0)=1;
        transform(1,1)=cos(ntheta);
        transform(1,2)=-sin(ntheta);
        transform(2,1)=sin(ntheta);
        transform(2,2)=cos(ntheta);
        transform(2,3)=cos(ntheta) * (-0.02) ;//* -0.001;
        transform(1,3)=-sin(ntheta) *  (-0.02) ;//* -0.001;
        transform(3,3) = 1;





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
        pt_out[2] = - pt_out[2];
        memcpy(&pointcloud_tmp.data[xyz_offset[2]],&pt_out[0], sizeof(float));
        memcpy(&(pointcloud_tmp.data[xyz_offset[0]]),&pt_out[1], sizeof(float));
        memcpy(&pointcloud_tmp.data[xyz_offset[1]],&pt_out[2], sizeof(float));

        xyz_offset += pointcloud_tmp.point_step;


    }

    ok++;
    //fout.close();


    pub.publish(pointcloud_tmp);

}





int main(int argc,char **argv)
{
    ros::init(argc,argv,"laser_transfor_new");
    ros::NodeHandle n;
    ROS_INFO("Now,start the node succeful!");

    fd=open("/dev/ttyUSB0",O_RDWR|O_NOCTTY|O_NDELAY);
    struct termios newtio,oldtio;
    bzero(&newtio,sizeof(newtio));
    newtio.c_cflag |=CLOCAL|CREAD;
    newtio.c_cflag &= ~CSIZE;

    newtio.c_cflag |= CS8;

    newtio.c_cflag &= ~PARENB;

    cfsetispeed(&newtio,B115200);
    cfsetospeed(&newtio,B115200);

    newtio.c_cflag |= CSTOPB;


    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd,TCIFLUSH);

    tcsetattr(fd,TCSANOW,&newtio);

    printf("fd = %d \n",fd);

    char begin_msg[]={'\xAA','\x55','\xFA','\x01','\xFB'};

    write(fd,begin_msg,5);
    sleep(2);

    pthread_t tid;
    int rv,t;
    t=1;



    pub=n.advertise<sensor_msgs::PointCloud2>("sync_scan_cloud_filtered",1);
    sub=n.subscribe("first",1,lCallback);


    ros::spin();
    thread_ok = false;
    //pthread_exit(NULL);
    char end_msg[]= {'\xAA','\x55','\xFA','\x02','\xFC'};
    write(fd,end_msg,5);

    return 0;
}



