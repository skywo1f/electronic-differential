#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#define PORT 8090 
#include <pthread.h>

#include <string>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

double x_pos = 5;

void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
//  std::cout << posesStamped->DebugString();

  ::google::protobuf::int32 sec = posesStamped->time().sec();
  ::google::protobuf::int32 nsec = posesStamped->time().nsec();
//  std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;
  for (int i =0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
//    if (name == std::string("my_velodyne")) {
      const ::gazebo::msgs::Vector3d &position = pose.position();

      double x = position.x();
      double y = position.y();
      double z = position.z();
	x_pos = x;
      std::cout << "Read position: x: " << x  << " y: " << y << " z: " << z << std::endl;
//    }
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
                printf("\nConnection Failed \n");
        }

        // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::setupClient(_argc, _argv);
#else
  gazebo::client::setup(_argc, _argv);
#endif


  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");


  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

          // Create our node for communication
//    gazebo::transport::NodePtr node2(new gazebo::transport::Node());
//    node2->Init();

    // Listen to Gazebo pose information topic
//         gazebo::transport::SubscriberPtr sub = node2->Subscribe("~/pose/info", posesStampedCallback);




	while(1){
                std::string hello_string = std::to_string(x_pos);
                char hello[hello_string.size() + 1];
                strcpy(hello, hello_string.c_str());
        char buffer[1024] = {0};
        send(sock , hello , strlen(hello) , 0 );
        printf("sent out ");
        printf(hello);
        printf("\n");
        valread = read( sock , buffer, 1024);
        printf("received ");
        printf("%s\n",buffer );
        int speed = atof(buffer);

  // Create a a vector3 message
  gazebo::msgs::Vector3d msg;

  // Set the velocity in the x-component
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::msgs::Set(&msg, gazebo::math::Vector3(speed, 0, 0));
#else
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(speed, 0, 0));
#endif

  // Send the message
  pub->Publish(msg);

  // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif

	}
}
