#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#define PORT 8191 
#include <pthread.h>
#include <vector>
#include <sstream>
#include <string>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/*
double x_pos_left = 0;
double z_pos_left = 0;
double x_pos_right = 0;
double z_pos_right = 0;
*/
//position
double x_pos = 0;
double y_pos = 0;
double z_pos = 0;
//orientation
double x_ori = 0;
double y_ori = 0;
double z_ori = 0;
double w_ori = 0;

void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
//  std::cout << posesStamped->DebugString();

  ::google::protobuf::int32 sec = posesStamped->time().sec();
  ::google::protobuf::int32 nsec = posesStamped->time().nsec();
  for (int i =0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
	if (name == std::string("my_follower")) {			//my_follower?
		const ::gazebo::msgs::Vector3d &position = pose.position();
		x_pos = position.x();
		y_pos = position.y();
		z_pos = position.z();

		const ::gazebo::msgs::Quaternion &orientation = pose.orientation();
		x_ori = orientation.x();
		y_ori = orientation.y();
		z_ori = orientation.z();
		w_ori = orientation.w();
//std::cout << "position x " << x_pos << "position z " << z_pos << std::endl;

	}


  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

        printf("started connecting to python\n");    
        int sock = 0, valread;
        struct sockaddr_in serv_addr;
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
                printf("\n Socket creation error \n");
        }
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        // Convert IPv4 and IPv6 addresses from text to binary form 
        if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
        {
                printf("\nInvalid address/ Address not supported \n");
        }
        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
                printf("\nConnection Failed for follower \n");
        }
gazebo::client::setup(_argc, _argv);
          // Create our node for communication
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        std::vector< double > wheelVec;
	double d = 0.0;
// Listen to Gazebo pose information topic
        gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", posesStampedCallback);


    while(1){
 
        std::string x_pos_string = std::to_string(x_pos);
	std::string y_pos_string = std::to_string(y_pos);
	std::string z_pos_string = std::to_string(z_pos);

	std::string x_ori_string = std::to_string(x_ori);
        std::string y_ori_string = std::to_string(y_ori);
        std::string z_ori_string = std::to_string(z_ori);
        std::string w_ori_string = std::to_string(w_ori);

	std::string all_strings = x_pos_string + " " + y_pos_string + " " + z_pos_string + " " + x_ori_string + " " + y_ori_string + " " + z_ori_string + " " + w_ori_string;
        char hello[all_strings.size() + 1];
        strcpy(hello, all_strings.c_str());
        char buffer[1024] = {0};
        send(sock , hello , strlen(hello) , 0 );
//        printf("request sent\n");
//        printf(hello);
        valread = read( sock , buffer, 1024);
	std::string bufferString(buffer);
        std::stringstream ss(bufferString);
        while (ss >> d)
            wheelVec.push_back (d);
//	std::cout << "wheel 1 " << wheelVec[0] << " wheel 2 " << wheelVec[1] << std::endl;

//	printf("\n%s\n",buffer );
//        printf("finished connecting to python\n");



  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  follow topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("~/my_follower/fol_cmd");

  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

  // Create a a vector3 message
  gazebo::msgs::Vector3d msg;

  // Set the velocity in the x-component
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::msgs::Set(&msg, gazebo::math::Vector3(wheelVec[0], 0, wheelVec[1]));
#else
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(wheelVec[0], 0, wheelVec[1]));
#endif
  wheelVec.clear();
  // Send the message
  pub->Publish(msg);
    
    }
}
