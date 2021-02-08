#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#define PORT 8090 
#include <pthread.h> 

typedef void * (*THREADFUNCPTR)(void *);

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoints()[0];
 
      this->joint2 = _model->GetJoints()[1];

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      this->pid2 = common::PID(0.1,0,0);
      
      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint2->GetScopedName(), this->pid2);


      // Default to zero velocity
      double velocity = 0;

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      this->SetVelocity(velocity);
      this->SetVelocity2(velocity);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &VelodynePlugin::OnMsgControl, this);
//                    //      pthread_create(&thread_id, NULL, this->startComms(), NULL);  



//      pthread_t thread_id;   
//      pthread_create(&thread_id, NULL, (THREADFUNCPTR) startComms(), NULL);
//      pthread_join(thread_id, NULL); 
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }


    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity2(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint2->GetScopedName(), _vel);
    }
	    
    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
      this->SetVelocity2(_msg->x());

    }


    private: void OnMsgControl(ConstVector3dPtr &_msg)
    {int speed = 10;
      if(_msg->x() == 2) {
        this->SetVelocity(speed);
        this->SetVelocity2(speed);
      }
            if(_msg->x() == 1) {
        this->SetVelocity(-speed);
        this->SetVelocity2(speed);
      }
      if(_msg->x() == 3) {
        this->SetVelocity(speed);
        this->SetVelocity2(-speed);
      }
      if(_msg->x() == 4) {
        this->SetVelocity(-speed);
        this->SetVelocity2(-speed);
      }


    }

    private: void *startComms() 
	     {
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
//	while(1) {
//	this->getDirection(sock,serv_addr);		
//	sleep(1);	
//	}
    }


    private: void getDirection(int sock, struct sockaddr_in serv_addr)
        {
	char *hello = "request";
	int valread;
	char buffer[1024] = {0};
	send(sock , hello , strlen(hello) , 0 );
	printf("request sent\n");
	valread = read( sock , buffer, 1024);
        printf("%s\n",buffer );
	int speed = atoi(buffer);
        this->SetVelocity(speed);
        this->SetVelocity2(speed);
	}



    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    private: physics::JointPtr joint2;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    private: common::PID pid2;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
