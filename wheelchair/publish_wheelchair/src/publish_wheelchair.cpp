#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <iostream>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>       // close()
#include <sys/socket.h>   // inet_addr()
#include <netinet/tcp.h>  // TCP_NODELAY
#include <arpa/inet.h>    // inet_addr()
#include <time.h>
#include "../include/publish_wheelchair/WheelchairData.h"

#include <errno.h>
#include <sys/time.h>
#include <sys/resource.h>

#define EMERGENCY_STOP_TIME 3
#define SOL_TCP 6
#define SERVER_PORT 65000
// #define IP_ADDR "192.168.1.87" // em16
using namespace std;

struct sockaddr_in saddr;

int clntFd;
MbSendData mbSendData;

void velocityCallback(const geometry_msgs::TwistStamped cmd_vel)
{
  int T, number;
  double Emergency;
  ros::Time ros_pc  = ros::Time::now();
  
  Emergency =  (ros_pc.sec + ros_pc.nsec*1.e-9) - (cmd_vel.header.stamp.sec + cmd_vel.header.stamp.nsec*1.e-9);

  if(Emergency < EMERGENCY_STOP_TIME){
    mbSendData.vel   = cmd_vel.twist.linear.x;
    mbSendData.omega = cmd_vel.twist.angular.z;
  }else{
    mbSendData.vel   = 0.;
    mbSendData.omega = 0.;
  }

  ROS_INFO_STREAM(
		  "Linear.x:" << mbSendData.vel << " "
		  "Angular.z:"<< mbSendData.omega
  );

  T = sendto(clntFd, &mbSendData, sizeof(MbSendData), 0,(struct sockaddr *)&saddr,sizeof(saddr));
  number = errno;
  if (T != sizeof(MbSendData)){
    printf("\033[2J");   /* 画面をクリア */
    printf("\033[1;1H"); /* カーソルを移動 */
    printf("%d\n", T);
    printf("%d,%lf,%lf\n", mbSendData.motionEndFlag, mbSendData.vel, mbSendData.omega);
    printf("%s\n",strerror(number));
    ROS_ERROR("send");
    flagamcl = 1;
  }

  return;
}

int main(int argc, char **argv)
{
  /* ソケット通信用 */
  int on = 1;
  int num= 0;
  int ChooseFlag = 0;
  char IP_ADDR[] = "192.168.1.86";
  flagamcl = 1;
  mbSendData.vel   = 0.0;
  mbSendData.omega = 0.0;

  /* ==========================================================
   ソケット通信開始:
  ========================================================== */
  do{
    printf("Preparing of connection to raspberry pi...\n");
    printf("Choose the color of raspberry pi.\n");
    printf("1.White\t2.Blue\t3.Red\t99.Manual\t\n");
    scanf("%d", &num);
    switch(num) {
      case 1:
          strcpy(IP_ADDR, "192.168.1.86");
          ChooseFlag = 1;
          break;
      case 2:
          strcpy(IP_ADDR, "192.168.1.87");
          ChooseFlag = 1;
          break;
      case 3:
          strcpy(IP_ADDR, "192.168.1.88");
          ChooseFlag = 1;
          break;
      case 99:
          printf("Type the ip address.\n");
          strcpy(IP_ADDR, "");
          printf("IP_ADDR =");
          scanf("%s", IP_ADDR);
          ChooseFlag = 1;
          break;
      default:
          printf("\033[2J");   /* 画面をクリア */
          printf("\033[1;1H"); /* カーソルを移動 */
          printf("Ok.Please select again.\n");
          ChooseFlag = 0;
    }

  }while(ChooseFlag == 0);
  printf("Your selected IP address is %s\n",IP_ADDR);

  /*ソケットの生成*/
  if ((clntFd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    ROS_ERROR("client: socket");
    return -1;
  }

  /*ソケットのオプションと設定を行う*/
  //setsockopt(clntFd, SOL_TCP, TCP_NODELAY, &on, sizeof(on));
  saddr.sin_family = AF_INET; //INETを使用
  saddr.sin_addr.s_addr = inet_addr(IP_ADDR); //通信先を指定
  saddr.sin_port = htons(SERVER_PORT); //通信に使用するポート番号

  /*接続*/

  ROS_INFO("socket connect succsess");
  /* ==========================================================
   cmd_velのSubscribe開始
  ========================================================== */

  ros::init(argc, argv, "ros2raspberry_pi");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/wheelchair/cmd_vel", 1000, velocityCallback);
  ros::spin();
  mbSendData.motionEndFlag = 1;

  if (sendto(clntFd, &mbSendData, sizeof(MbSendData), 0,(struct sockaddr *)&saddr,sizeof(saddr)) != sizeof(MbSendData)){
    ROS_ERROR("send");
  }

  close(clntFd);

  return 0;
}
