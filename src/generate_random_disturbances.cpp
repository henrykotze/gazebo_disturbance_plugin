#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math/Vector3.hh>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>

#include <ros/ros.h>

#include <sensor_msgs/TimeReference.h>


#include <sdf/sdf.hh>

namespace gazebo
{
  class DisturbedModel : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }


      if (_sdf->HasElement("max_xy_force")){

        max_xy_force = _sdf->GetElement("max_xy_force")->Get<double>();
      }
      else{
        // gzerr << "[gazebo_disturbance_plugin] please specify max_xy_force\n";
        ROS_FATAL_STREAM("[gazebo_disturbance_plugin] please specify max_xy_force\n");

      }

      if (_sdf->HasElement("max_z_force")){
        max_z_force = _sdf->GetElement("max_z_force")->Get<double>();
      }
      else{
        gzerr << "[gazebo_disturbance_plugin] please specify max_z_force \n";

      }

      if (_sdf->HasElement("max_z_moment")){
        max_z_moment = _sdf->GetElement("max_z_moment")->Get<double>();
      }
      else{
        gzerr << "[gazebo_disturbance_plugin] please specify max_z_moment\n";

      }

      if (_sdf->HasElement("max_xy_moment")){
        max_xy_moment = _sdf->GetElement("max_xy_moment")->Get<double>();
      }
      else{
        gzerr << "[gazebo_disturbance_plugin] please specify max_xy_moment\n";

      }


      // ros NodeHandle
      ros::NodeHandle nh;

      // Store the pointer to the model
      this->model = _parent;
      // this->world = this->model->GetWorld();


      drone_model = model->GetChildLink("base_link");


      // explanation: https://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
      subscriber = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, boost::bind(&DisturbedModel::flight_state_cb,this,_1));
      subscriber = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, boost::bind(&DisturbedModel::drone_state_cb,this,_1));


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DisturbedModel::OnUpdate, this));

     // gzmsg << "[Gazebo Disturbance Plugin] Loaded" << "\n";
      ROS_INFO("Gazebo Disturbance Plugin LOADED");

    }



    // Called by the world update start event
    public: void OnUpdate()
    {
      // check callbacks
       ros::spinOnce();


      // ROS_INFO("Drone local position is: %f, %f, %f",drone_state.pose.position.x,drone_state.pose.position.y,drone_state.pose.position.z);

      // if the drone is in the AIR or LANDING, do disturbance when drone > 0.1m from ground
      if( (flight_state.landed_state == 2 || flight_state.landed_state == 4 ) && drone_state.pose.position.z > 0.1){
        ROS_INFO("DRONE IS IN AIR & activating disturbances");
        // drone_model->AddRelativeForce(force_disturbances[0]);
        // drone_model->AddRelativeTorque(torque_disturbances[0]);
        // drone_model->AddRelativeForce(ignition::math::Vector3d(0,0,2));
      }


      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }


    private: void generate_disturbance()
    {

    }

    // callback to retrieve state
    public: void flight_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
    {
      flight_state = *msg;
    }


    // callback to retrieve state
    public: void drone_state_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      drone_state = *msg;
    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // flying state of drone
    private: mavros_msgs::ExtendedState flight_state;

    // state of drone
    private: geometry_msgs::PoseStamped drone_state;

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

    private: double max_xy_force = 0;
    private: double max_z_force = 0;
    private: double max_z_moment = 0;
    private: double max_xy_moment = 0;



  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DisturbedModel)
}
