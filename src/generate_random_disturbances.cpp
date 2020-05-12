#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>

#include <ros/ros.h>

#include <sensor_msgs/TimeReference.h>

#include <sdf/sdf.hh>

#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <math.h>       /* exp */

#include <ctime>

std::vector<double> generateRandomPulseTrain(int lenght_of_signal, double minInput, double maxInput);
std::vector<double> generateRandomRampTrains(int lenght_of_signal, double minInput, double maxInput);
std::vector<double> generateRandomExpoTrains(int lenght_of_signal, double minInput, double maxInput);
std::vector<double> generateRandomSquareTrains(int lenght_of_signal, double minInput, double maxInput);


template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

void saveDisturbance2CSV(std::vector<double> timestamps,std::vector<ignition::math::Vector3d> force_disturb,std::vector<ignition::math::Vector3d> moment_disturb,std::string file_path);

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
        return;

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

      if (_sdf->HasElement("disturbance_time")){
        disturbance_time = (_sdf->GetElement("disturbance_time")->Get<int>());
        disturbance_time = 100*250;
      }
      else{
        disturbance_time = 0;

      }


      // ros NodeHandle
      ros::NodeHandle nh;

      // Store the pointer to the model
      this->model = _parent;
      // this->world = this->model->GetWorld();


      drone_model = model->GetChildLink("base_link");


      // explanation: https://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
      sub1 = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, boost::bind(&DisturbedModel::flight_state_cb,this,_1));
      sub2 = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, boost::bind(&DisturbedModel::drone_state_cb,this,_1));
      sub3 = nh.subscribe<sensor_msgs::TimeReference>("mavros/time_reference", 100, boost::bind(&DisturbedModel::time_cb,this,_1));
      sub4 = nh.subscribe<mavros_msgs::State>("mavros/state", 1, boost::bind(&DisturbedModel::flight_status_cb,this,_1));


      // generate the random disturbance sequence to influence the system
      generate_disturbances_sequence();


      std::time_t log_filename = time(nullptr);
      std::ostringstream name_of_logfile;
      name_of_logfile << "/home/henry/esl-sun/PX4/build/px4_sitl_default/logs/" << std::put_time(std::gmtime(&log_filename), "%Y-%m-%e/%H_%M_%S_disturbance.csv");
      logfile_name = name_of_logfile.str();
      ROS_INFO_STREAM_ONCE("log_file: " << logfile_name);


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DisturbedModel::OnUpdate, this));

     // gzmsg << "[Gazebo Disturbance Plugin] Loaded" << "\n";
      ROS_INFO("Gazebo Disturbance Plugin LOADED");


      // save disturbance
      saveDisturbance2CSV(timestamps,force_disturbances,torque_disturbances,logfile_name);



    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      // check callbacks
       ros::spinOnce();


       if(disturbance_active == 0){
         if(ros::Time::now().sec >= 20){
           disturbance_active = 1;
         }
       }

      if(disturbance_active == 1){
        ROS_INFO_ONCE("DISTURBANCE ACTIVATED");

        // if( (flight_state.landed_state == 2 || flight_state.landed_state == 4 ) && drone_state.pose.position.z > 0.1){

            drone_model->AddRelativeForce(force_disturbances[counter]);
            drone_model->AddRelativeTorque(torque_disturbances[counter]);

        // }
        counter++;


       }



    }


    private: void generate_disturbances_sequence()
    {
      std::vector<double> fx = generateRandomPulseTrain(disturbance_time, -max_xy_force, max_xy_force);
      std::vector<double> fy = generateRandomPulseTrain(disturbance_time, -max_xy_force, max_xy_force);
      std::vector<double> fz = generateRandomPulseTrain(disturbance_time, -max_z_force, max_z_force);

      // std::vector<double> fx = generateRandomPulseTrain(disturbance_time, 0,0);
      // std::vector<double> fy = generateRandomPulseTrain(disturbance_time, 0,0);
      // std::vector<double> fz = generateRandomPulseTrain(disturbance_time, 0,0);



      std::vector<double> mx = generateRandomPulseTrain(disturbance_time, -max_xy_moment, max_xy_moment);
      std::vector<double> my = generateRandomPulseTrain(disturbance_time, -max_xy_moment, max_xy_moment);
      std::vector<double> mz = generateRandomPulseTrain(disturbance_time, -max_z_moment, max_z_moment);


      // std::vector<double> mx = generateRandomPulseTrain(disturbance_time, 0,0);
      // std::vector<double> my = generateRandomPulseTrain(disturbance_time, 0,0);
      // std::vector<double> mz = generateRandomPulseTrain(disturbance_time, 0,0);

      for(int x = 0; x < disturbance_time; x++){

        force_disturbances.push_back(ignition::math::Vector3d(fx[x],fy[x],fz[x]));
        torque_disturbances.push_back(ignition::math::Vector3d(mx[x],my[x],mz[x]));

      }

      timestamps = linspace(20.0,120.0,(size_t)25000);


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


    // callback to retrieve state
    public: void time_cb(const sensor_msgs::TimeReference::ConstPtr& msg)
    {
      px4_time = *msg;
    }

    // callback to retrieve state
    public: void flight_status_cb(const mavros_msgs::State::ConstPtr& msg)
    {
      flight_status = *msg;
    }



    // Pointer to the model
    private: physics::ModelPtr model;

    private: int disturbance_active = 0;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // flying state of drone
    private: mavros_msgs::ExtendedState flight_state;

    // state of drone
    private: geometry_msgs::PoseStamped drone_state;

    // Flight status
    private: mavros_msgs::State flight_status;


    private: double max_xy_force = 0;
    private: double max_z_force = 0;
    private: double max_z_moment = 0;
    private: double max_xy_moment = 0;
    private: double start_time_disturbance = 0;
    private: int disturbance_time;
    private: long unsigned int counter = 0;
    private: std::string logfile_name;

    // force disturbances effecting the drone
    private: std::vector<ignition::math::Vector3d> force_disturbances;

    // moment disturbances effecting the drone
    private: std::vector<ignition::math::Vector3d> torque_disturbances;

    // timestamps for disturbances
    private: std::vector<double> timestamps;

    // link that will experience the disturbance
    private: physics::LinkPtr drone_model;

    // require some logging variable

    // time variable
    private:  sensor_msgs::TimeReference px4_time;

    // ros subscriber
    private: ros::Subscriber sub1;
    private: ros::Subscriber sub2;
    private: ros::Subscriber sub3;
    private: ros::Subscriber sub4;




  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DisturbedModel)
}


void saveDisturbance2CSV(std::vector<double> timestamps,std::vector<ignition::math::Vector3d> force_disturb,std::vector<ignition::math::Vector3d> moment_disturb,std::string file_path){

    std::ofstream csvFile(file_path);

    // create column names for csv file
    csvFile << "time,fx,fy,fz,mx,my,mz\n";

    for(int i = 0; i < force_disturb.size(); i++)
    {
        //still to do: Add time vector
        csvFile << timestamps.at(i);
        csvFile << ",";
        csvFile << force_disturb.at(i).X() << "," << force_disturb.at(i).Y() << "," << force_disturb.at(i).Z();
        csvFile << ",";
        csvFile << moment_disturb.at(i).X() << "," << moment_disturb.at(i).Y() << "," << moment_disturb.at(i).Z();
        csvFile << "\n";
    }

    // Close the file
    csvFile.close();

}


std::vector<double> generateRandomPulseTrain(int lenght_of_signal, double minInput, double maxInput){

  std::vector<double> signal(lenght_of_signal,0.0);
  double magInput = 0;
  int inputDur = 0;     // time of which there is a step
  int zeroInputDur = 0; // time of which there is a zero input

  std::vector<double>::iterator i = signal.begin();

  while(i < signal.end()){

    magInput = ignition::math::Rand::DblUniform(minInput,maxInput); // magnitude of input
    inputDur = (int)(ignition::math::Rand::DblUniform(0,1)*(lenght_of_signal/10)); // time of magnitudel
    zeroInputDur = (int)(ignition::math::Rand::DblUniform(0,1)*(lenght_of_signal/10)); // time of magnitude

    if(i + inputDur + zeroInputDur > signal.end()){
        break;
    }
    else{

      std::fill(i, i + inputDur, magInput);
      i = i + inputDur + zeroInputDur;
    }
  }

  return signal;

}




std::vector<double> generateRandomRampTrains(int lenght_of_signal, double minInput, double maxInput){

  std::vector<double> signal(1,0.0);
  double maxPoint = 0;
  int Dur = 0;     // time of which there is a step
  int zeroDur = 0; // time of which there is a zero input
  double grad= 0;

  std::vector<double>::iterator i = signal.begin();
  int counter = 0;

  while(counter < lenght_of_signal){

    maxPoint = ignition::math::Rand::DblUniform(minInput,maxInput); // magnitude of input
    Dur = (int)(ignition::math::Rand::DblUniform(0,1)*(lenght_of_signal/10)); // time of first line
    zeroDur = (int)(ignition::math::Rand::DblUniform(0,1)*(lenght_of_signal/10)); // time of second line

    std::vector<double> Line(Dur);

    if(counter + Dur + zeroDur > lenght_of_signal){
        break;
    }
    else{

      grad = maxPoint/(double)Dur;
      boost::range::push_back(Line, boost::irange(0,Dur));
      //
      std::transform(Line.begin(), Line.end(), Line.begin(),std::bind(std::multiplies<double>(), std::placeholders::_1, grad));

      signal.insert(signal.end(),Line.begin(),Line.end());
      signal.insert(signal.end(),zeroDur,0.0);

      counter = counter + Dur + zeroDur;
    }
  }

  return signal;
}


std::vector<double> generateRandomExpoTrains(int lenght_of_signal, double minInput, double maxInput){

  std::vector<double> signal(1,0.0);

  int Dur = 0;     // time of which there is a expo
  int zeroDur = 0; // time of which there is a zero input

  double alpha = 0;
  double constant = 0;
  double val = 0;


  std::vector<double>::iterator i = signal.begin();
  int counter = 0;

  while(counter < lenght_of_signal){


    Dur = (int)(ignition::math::Rand::DblUniform(0,1)*(lenght_of_signal/10)); // time of first line
    zeroDur = (int)(ignition::math::Rand::DblUniform(0,1)*(lenght_of_signal/10)); // time of second line
    alpha = ignition::math::Rand::DblUniform(-0.05,0.05); // magnitude of input
    constant = ignition::math::Rand::DblUniform(-0.5,0.5);
    // constant = 0.001;

    std::vector<double> Line(Dur);

    if(counter + Dur + zeroDur > lenght_of_signal){
        break;
    }
    else{

      boost::range::push_back(Line, boost::irange(0,Dur));

      for(int i = 0; i < Line.size(); i++){
        val = constant*exp(alpha*i);

        if(val > maxInput){
          Line[i] = 0;
        }
        else if(val < minInput){
          Line[i] = 0;
        }
        else{
          Line[i] = val;
        }

      }

      signal.insert(signal.end(),Line.begin(),Line.end());
      signal.insert(signal.end(),zeroDur,0.0);
      counter = counter + Dur + zeroDur;
    }
  }

  return signal;
}



std::vector<double> generateRandomSquareTrains(int lenght_of_signal, double minInput, double maxInput){

  std::vector<double> signal(1,0.0);

  int Dur = 0;     // time of which there is a expo
  int zeroDur = 0; // time of which there is a zero input

  double a = 0;
  double b = 0;
  double c = 0;
  double val = 0;


  std::vector<double>::iterator i = signal.begin();
  int counter = 0;

  while(counter < lenght_of_signal){


    Dur = (int)(ignition::math::Rand::DblUniform(0,1)*(lenght_of_signal/10)); // time of first line
    zeroDur = (int)(ignition::math::Rand::DblUniform(0,1)*(lenght_of_signal/10)); // time of second line
    a = ignition::math::Rand::DblUniform(-0.001,0.001); // magnitude of input
    b = ignition::math::Rand::DblUniform(-0.001,0.001);
    c = ignition::math::Rand::DblUniform(-0.001,0.001);
    // constant = 0.001;

    std::vector<double> Line(Dur);

    if(counter + Dur + zeroDur > lenght_of_signal){
        break;
    }
    else{

      boost::range::push_back(Line, boost::irange(0,Dur));

      for(int i = 0; i < Line.size(); i++){
        val = a*pow(i,2) + b*i + c;

        if(val > maxInput){
          Line[i] = 0;
        }
        else if(val < minInput){
          Line[i] = 0;
        }
        else{
          Line[i] = val;
        }

      }

      signal.insert(signal.end(),Line.begin(),Line.end());
      signal.insert(signal.end(),zeroDur,0.0);
      counter = counter + Dur + zeroDur;
    }
  }

  return signal;
}
