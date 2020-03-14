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

namespace gazebo
{
  class DisturbedModel : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      ROS_INFO("LOADED");


      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DisturbedModel::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }


    private: void generate_disturbance(){

    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // state of drone
    private: mavros_msgs::ExtendedState current_state;

    // force disturbances effecting the drone
    private: std::vector<ignition::math::Vector3d> force_disturbances;

    // moment disturbances effecting the drone
    private: std::vector<ignition::math::Vector3d> torque_disturbances;


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DisturbedModel)
}
