#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <iostream>

#include <string.h>       // bzero()
#include <stdlib.h>
#include <time.h>
#include <netdb.h>        // gethostbyname()
#include <unistd.h>       // close()
#include <sys/socket.h>   // inet_addr()
#include <netinet/in.h>   // inet_addr()
#include <netinet/tcp.h>  // TCP_NODELAY
#include <arpa/inet.h>    // inet_addr()
#include <curses.h>
#include "../include/subscribe_wheelchair/WheelchairData.h"

#define SERVER_PORT 65500
#define PUB_RATE 50000
#define DT_SOCK 0.02

extern int clock_gettime(clockid_t clk_id, struct timespec *res);

using namespace std;

int main(int argc, char** argv)
{
  /* ソケット通信用 */
  struct sockaddr_in saddr;
  socklen_t len = sizeof(struct sockaddr_in);
  int i;
  int on;
  int servFd;
  int clntFd;
  int rsize;
  MbRecvData mbRecvData = {0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

  /*時間関係の変数群*/
  struct timespec interT;
  int interbal = DT_SOCK * 1000000000; //[ns]
  double fastTime = 0.0;
  double interTime = 0.0;
  double oldInterTime = 0.0;
  double deltaT = 0.0;

 /* ==========================================================bzero
     ソケット通信の初期化:
   ========================================================== */
  /* ソケットの生成 */
  if ((servFd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    ROS_ERROR("socket");
    exit(0);
  }
  ROS_INFO("Open server.");
  on = 1;

  /* saddrの中身を0にしておかないと、bind()でエラーが起こることがある*/
  bzero((char *)&saddr, sizeof(saddr));
  /* ソケットにアドレスとポートを結びつける */
  saddr.sin_family        = AF_INET;
  saddr.sin_addr.s_addr   = INADDR_ANY;
  saddr.sin_port          = htons(SERVER_PORT);

  if (bind(servFd, (struct sockaddr *)&saddr, len) < 0) {
    ROS_ERROR("bind");
    exit(0);
  }

 /* ==========================================================
     odometry_publisherの初期化:
   ========================================================== */

  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/wheelchair/odom", 1000);
  ros::Publisher gyr_pub = n.advertise<geometry_msgs::Point>("/wheelchair/gyr", 100);
  ros::Publisher measure_pub = n.advertise<geometry_msgs::Point>("/measure_odom", 1000);
  ros::Publisher acc_pub = n.advertise<geometry_msgs::Point>("/wheelchair/acc", 1000);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  double wheelRVel = 0.0;
  double wheelLVel = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  double ct = ros::Time::now().toSec();

  ros::Rate r(PUB_RATE);

  /*double vell[100000];
    double velr[100000];
    double time[100000];*/
  //time[0] = 0;
    double timekeeper;
    double lt = ct;
    timekeeper = 0;
 /* ==========================================================
     ソケット通信でデータを受け取り＆/odomを配信:
   ========================================================== */
  clock_gettime( 0, &interT );
  fastTime = (double)interT.tv_sec + (double)interT.tv_nsec * 0.000000001;
  while(n.ok()){
    double lt = ros::Time::now().toSec();
    interTime = (double)interT.tv_sec + (double)interT.tv_nsec * 0.000000001;
    rsize = recvfrom(servFd, (char *)&mbRecvData, sizeof(mbRecvData), 0,(struct sockaddr *)&saddr,(socklen_t *)sizeof(saddr));
    deltaT = interTime - oldInterTime;
    printf("deltaT: %lf\n", deltaT);
    oldInterTime = interTime;

    if(flagamcl == 5) {
      ROS_INFO("socketServ end");
      break;
    }

    wheelRVel = -mbRecvData.vx;
    wheelLVel = mbRecvData.vy;
    printf("gyr X:%f, Y:%f, Z:%f\n",mbRecvData.gyrX/1000*3.14159265/180.0,mbRecvData.gyrY/1000*3.14159265/180.0,mbRecvData.gyrZ/1000*3.14159265/180.0);
    printf("acc X:%f, Y:%f, Z:%f\n" ,mbRecvData.accX,mbRecvData.accY,mbRecvData.accZ);
    /*if(i <= 100000){
      vell[i] = wheelLVel;
      velr[i] = wheelRVel;
      time[i] =  time[i-1]+timekeeper;;
      }*/
    oldInterTime = lt;
    //vth = mbRecvData.vth;

    vth = (wheelRVel - wheelLVel) / 0.49;
    //th += vth * 0.000002;
    th += vth * 0.00000002;
    vx = (wheelRVel + wheelLVel) * cos(th) / 2;
    vy = (wheelRVel + wheelLVel) * sin(th) / 2;


    ROS_INFO_STREAM(
	"wheelR:"<< wheelRVel <<
    "th:"<< th <<
    " vx:"<< vx <<
    " vy:"<< vy <<
    " vth:"<< vth);

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double delta_x = vx * dt;
    double delta_y = vy * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "wheelchair/odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "wheelchair/odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = vth;
    odom.twist.twist.angular.x = wheelLVel;
    odom.twist.twist.angular.y = wheelRVel;
    odom.twist.twist.angular.z = dt;

    //set the odometry
    geometry_msgs::Point measure_odom;
    measure_odom.x = wheelLVel;
    measure_odom.y = wheelRVel;
    measure_odom.z = dt;

    //publish the message
    odom_pub.publish(odom);
    measure_pub.publish(measure_odom);

    geometry_msgs::Point gyr;

    gyr.x = mbRecvData.gyrX/1000*3.14159265/180.0;
    gyr.y = mbRecvData.gyrY/1000*3.14159265/180.0;
    gyr.z = mbRecvData.gyrZ/1000*3.14159265/180.0;

    gyr_pub.publish(gyr);

    geometry_msgs::Point acc;

    acc.x = mbRecvData.accX/1000;
    acc.y = mbRecvData.accY/1000;
    acc.z = mbRecvData.accZ/1000;

    acc_pub.publish(acc);

    last_time = current_time;

    i++;

    r.sleep();

  // }if(i <= 100000){
	 //  printf("fin");
	 //  FILE *fpwl =  fopen("odometoryl.txt","w");
	 //  for(i=1;i<=100000;i++){
		//   fprintf(fpwl,"%lf\n",vell[i]);
	 //  }
	 //  fclose(fpwl);
	 //  FILE *fpwr =  fopen("odometoryr.txt","w");
	 //  for(i=1;i<=100000;i++){
		//   fprintf(fpwr,"%lf\n",velr[i]);
	 //  }
	 //  fclose(fpwr);
	 //  FILE *fpwt =  fopen("time.txt","w");
	 //  for(i=1;i<=100000;i++){
		//   fprintf(fpwt,"%lf\n",time[i]);
	 //  }
	 //  fclose(fpwt);
  }

  close(clntFd);
  ROS_INFO("socket closed");
  return 0;
}
