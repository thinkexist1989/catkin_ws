#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <std_msgs/String.h>
#include <netdb.h>
#include <arpa/inet.h>


int main(int argc, char** argv)
{
	ros::init(argc,argv,"tcp_node");
	ros::NodeHandle nh;
	ros::Publisher tcp_node = nh.advertise<std_msgs::String>("tcp_message",1000);
	
	int sockfd,portno,n;
	socklen_t clilen;
	char buffer[256];
	struct sockaddr_in serv_addr;
	struct hostent *server;
	std_msgs::String msg;
	
	
        portno = 6666;

	sockfd = socket(AF_INET,SOCK_STREAM,0);
	if(sockfd < 0){
		perror("ERROR opening socket");	
	}
	std::cout << "open socket success!" <<std::endl;
	//server = gethostbyname(argv[1]);
	std::cout <<"gethostbyname success!"<<std::endl;
	bzero(&serv_addr,sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	//inet_aton(argv[1],(struct in_addr*)&serv_addr.sin_addr.s_addr);
    	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);

	serv_addr.sin_port = htons(portno);
	std::cout <<"serv_addr success!"<<std::endl;
        if(connect(sockfd,(struct sockaddr*)&serv_addr,sizeof(serv_addr)) < 0){
		perror("ERROR connecting");
            return 0;
        }
	
	std::cout <<"connecting success!"<<std::endl;
	
	while(ros::ok()){

		bzero(buffer,256);
                //char sendbuf[] = "hello world";
                //n = write(sockfd,sendbuf,12);
                n = recv(sockfd,buffer,108,0);
		if(n < 0)
			perror("ERROR reading reply");
		std::stringstream ss;
                ss << "I heard: " << buffer[0] << buffer[1];

		msg.data = ss.str();
		tcp_node.publish(msg);
		ROS_INFO("%s",msg.data.c_str());
		
		ros::spinOnce();	
	}
}
