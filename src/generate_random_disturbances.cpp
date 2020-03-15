#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>

namespace gazebo
{
  class DisturbedModel : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_ros", ros::init_options::NoSigintHandler |
                                        ros::init_options::AnonymousName);
      }

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      ROS_INFO("LOADED");


      // ros NodeHandle
      ros::NodeHandle nh;

      // Store the pointer to the model
      this->model = _parent;

      drone_model = model->GetChildLink("base_link");


      // explanation: https://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
      subscriber = nh.subscribe<mavros_msgs::ExtendedState>("mavros/ExtendedStates", 10, boost::bind(&DisturbedModel::state_cb,this,_1));


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DisturbedModel::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // check callbacks
      ros::spinOnce();

      // if the drone is in the air
      if(drone_state.landed_state == 2){
        ROS_INFO("DRONE IS IN AIR");
        // drone_model->AddRelativeForce(force_disturbances[0]);
        // drone_model->AddRelativeTorque(torque_disturbances[0]);
        drone_model->AddRelativeForce(ignition::math::Vector3d(0,0,-2));
      }

      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }


    private: void generate_disturbance()
    {

    }

    // callback to retrieve state
    public: void state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
    {
      drone_state = *msg;
    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // state of drone
    private: mavros_msgs::ExtendedState drone_state;

    // force disturbances effecting the drone
    private: std::vector<ignition::math::Vector3d> force_disturbances;

    // moment disturbances effecting the drone
    private: std::vector<ignition::math::Vector3d> torque_disturbances;

    // link that will experience the disturbance
    private: physics::LinkPtr drone_model;

    // require some logging variable

    // time variable
    private:  sensor_msgs::TimeReference px4_time;

    // ros subscriber
    private: ros::Subscriber subscriber;



  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DisturbedModel)
}
