#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <std_msgs/String.h>
#include <tcp_test/motor.h>
#include <tcp_test/sensor.h>
#include <tcp_test/platform.h>
#include <tcp_test/lightctrl.h>
#include <tcp_test/motorctrl.h>
#include <tcp_test/platformctrl.h>

#define TCP_RECIEVE_LEN 108
#define TCP_SEND_LEN    56

int sockfd,portno,n;
socklen_t clilen;
struct sockaddr_in serv_addr;
struct hostent *server;
std_msgs::String msg;
tcp_test::motor  motormsg;
tcp_test::sensor sensormsg;
tcp_test::platform platmsg;


void motorctrlCallback(const tcp_test::motorctrl::ConstPtr& msg);
void platformctrlCallback(const tcp_test::platformctrl::ConstPtr& msg);
void lightctrlCallback(const tcp_test::lightctrl::ConstPtr& msg);

int main(int argc, char** argv)
{
  ros::init(argc,argv,"uwv_tcp_node");
	ros::NodeHandle nh;
  ros::Publisher motor = nh.advertise<tcp_test::motor>("motor",1000);  // publish /motor topic
  ros::Publisher sensor = nh.advertise<tcp_test::sensor>("sensor",1000); // publish /sensor topic
  ros::Publisher platform = nh.advertise<tcp_test::platform>("platform",1000); //publish /platform topic
	
  ros::Subscriber motorctrl = nh.subscribe("motorctrl",1000,motorctrlCallback); // subscribe /motorctrl topic including msg of pwm
  ros::Subscriber platformctrl = nh.subscribe("platformctrl",1000,platformctrlCallback); // subscribe /platformctrl topic including msg of 3-axis and feeding motors
  ros::Subscriber lightctrl = nh.subscribe("lightctrl",1000,lightctrlCallback); // subscribe /lightctrl topic to set underwater lights' brightness



  portno = 6666;
  sockfd = socket(AF_INET,SOCK_STREAM,0);
	if(sockfd < 0){
		perror("ERROR opening socket");	
	}
	std::cout << "open socket success!" <<std::endl;
	bzero(&serv_addr,sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
  // serv_addr.sin_addr.s_addr = inet_addr("192.168.0.1");
	serv_addr.sin_port = htons(portno);
  if(connect(sockfd,(struct sockaddr*)&serv_addr,sizeof(serv_addr)) < 0){
		perror("ERROR connecting");
    return -1;
  }
	std::cout <<"connecting success!"<<std::endl;

  char recvbuf[TCP_RECIEVE_LEN];
	
	while(ros::ok()){

    bzero(recvbuf,TCP_RECIEVE_LEN);
  //	char sendbuf[] = "hello world";
  //	n = write(sockfd,sendbuf,12);
    n = recv(sockfd,recvbuf,TCP_RECIEVE_LEN,0);
    if(n != TCP_RECIEVE_LEN) {
    //	perror("ERROR reading reply");
      continue;
    }
    if(recvbuf[3] != 0){
      for(int i = 0; i< 8;i++){
        motormsg.motor[i].ID = i;
        memcpy(&motormsg.motor[i].speed,&recvbuf[4+i*4],2);
        memcpy(&motormsg.motor[i].current,&recvbuf[6+i*4],2);
      }
      motor.publish(motormsg);
      ROS_INFO("receive motor info, motor1 speed is: %d",motormsg.motor[0].speed);
    }

    if(recvbuf[36] != 0){  //xsens mti-g-700 data   euler angle   float
      sensormsg.xsens.ID = 0;
      memcpy(&sensormsg.xsens.roll, &recvbuf[37],4);
      memcpy(&sensormsg.xsens.pitch,&recvbuf[41],4);
      memcpy(&sensormsg.xsens.yaw,  &recvbuf[45],4);
    }

    if(recvbuf[49] != 0){  //keller pressure data   float
      sensormsg.keller.ID = 0;
      memcpy(&sensormsg.keller.value, &recvbuf[50],4);
    }

    if(recvbuf[54] != 0){  //altimeter data         float
      for(int i = 0;i < 2;i++){
        sensormsg.altimeter[i].ID = i;
        memcpy(&sensormsg.altimeter[i].distance   ,&recvbuf[55+12*i],4);
        memcpy(&sensormsg.altimeter[i].energy     ,&recvbuf[55+12*i],4);
        memcpy(&sensormsg.altimeter[i].correlation,&recvbuf[55+12*i],4);
      }
    }

    memcpy(&sensormsg.temp_cabin,  &recvbuf[79], 4);  // cabin temprature
    memcpy(&sensormsg.temp_wall ,  &recvbuf[83], 4);  // wall  temprature
    memcpy(&sensormsg.temp_water,  &recvbuf[87], 4);  // water temprature

    sensor.publish(sensormsg);
    ROS_INFO("roll:%.2f,pitch:%.2f,yaw:%.2f,press:%.3f,alt:%.3f,%.3f",sensormsg.xsens.roll,sensormsg.xsens.pitch,sensormsg.xsens.yaw,sensormsg.keller.value,sensormsg.altimeter[0].distance,sensormsg.altimeter[1].distance);

    if(recvbuf[91] != 0){
      memcpy(&platmsg.axis_x,       &recvbuf[92], 4); //axis-x
      memcpy(&platmsg.axis_y,       &recvbuf[96], 4); //axis-y
      memcpy(&platmsg.axis_z,       &recvbuf[100], 4); //axis-z
      memcpy(&platmsg.feeding_speed,&recvbuf[104], 4); //feeding-speed

      platform.publish(platmsg);
    }

		ros::spinOnce();	
	}
}

void motorctrlCallback(const tcp_test::motorctrl::ConstPtr& msg)
{
  ROS_INFO("motor control, motor1 pwm is: %f",msg->pwm[0]);
  char sendbuf[TCP_SEND_LEN] = {0};
  sendbuf[0] = 0xFF; sendbuf[1] = 0xFE; sendbuf[3] = 0x11;
  sendbuf[4] = msg->index;
  memcpy(&sendbuf[4],&(msg->pwm),32);
  n = send(sockfd,sendbuf,TCP_SEND_LEN,0);
}

void platformctrlCallback(const tcp_test::platformctrl::ConstPtr& msg)
{
  char sendbuf[TCP_SEND_LEN] = {0};
  sendbuf[0] = 0xFF; sendbuf[1] = 0xFE; sendbuf[3] = 0x22;
  sendbuf[36] = 0x01;
  memcpy(&sendbuf[37],&(msg->axis_x),4);
  memcpy(&sendbuf[37],&(msg->axis_y),4);
  memcpy(&sendbuf[37],&(msg->axis_z),4);
  memcpy(&sendbuf[37],&(msg->feeding_speed),4);
  n = send(sockfd,sendbuf,TCP_SEND_LEN,0);
}

void lightctrlCallback(const tcp_test::lightctrl::ConstPtr& msg)
{
  char sendbuf[TCP_SEND_LEN] = {0};
  sendbuf[0] = 0xFF; sendbuf[1] = 0xFE; sendbuf[3] = 0x33;
  sendbuf[53] = msg->index;
  memcpy(&sendbuf[54],&(msg->brightness),2);
  n = send(sockfd,sendbuf,TCP_SEND_LEN,0);
}
