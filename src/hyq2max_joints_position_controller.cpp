#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <control_toolbox/pid.h>
#include <hyq2max_joints_position_controller/HyQ2max_joints.h>
#include <hyq2max_joints_position_controller/HyQ2max_command.h>


namespace hyq2max_joints_position_controller_ns{

class HyQ2maxJointsPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    /*
    std::string my_joint;
    if (!n.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }*/

    // get the joint object to use in the realtime loop
    //std::string lf_joints[3];

    /*
    std::vector<std::string> lf_joints;
    std::vector<std::string> lh_joints;
    std::vector<std::string> rh_joints;
    std::vector<std::string> rf_joints;
    n.getParam("left_front_joints", lf_joints);
               //"left_front_joints"
    //std::string lh_joints[3];
    n.getParam("left_hind_joints", lh_joints);
    //std::string rh_joints[3];
    n.getParam("right_hind_joints", rh_joints);
    //std::string rf_joints[3];
    n.getParam("right_front_joints", rf_joints);
/*
    ROS_INFO("%s", lf_joints);
    ROS_INFO("%s", lh_joints);
    ROS_INFO("%s", rh_joints);
    ROS_INFO("%s", rf_joints);

    */

    XmlRpc::XmlRpcValue lf_joints, lh_joints, rh_joints, rf_joints;
    n.getParam("left_front_joints", lf_joints);
    n.getParam("left_hind_joints", lh_joints);
    n.getParam("right_hind_joints", rh_joints);
    n.getParam("right_front_joints", rf_joints);

    joints.resize(12);
    for (int i = 0; i < 3; i++){
      joints[i] = hw->getHandle(lf_joints[i]);
      joints[i+3] = hw->getHandle(lh_joints[i]);
      joints[i+6] = hw->getHandle(rh_joints[i]);
      joints[i+9] = hw->getHandle(rf_joints[i]);
    }
/*
    for (int i = 0; i < 3; i++){
      //ROS_INFO("%s", lf_joints[i]);
      std::cout << lf_joints[i] << "\n";
      /*ROS_INFO("%s", lf_joints[i]);
      ROS_INFO("%s", lh_joints[i]);
      ROS_INFO("%s", rh_joints[i]);
      ROS_INFO("%s", rf_joints[i]);
    }
    */
     // Load PID Controller using gains set on parameter server
  	//if (!pid_controller.init(ros::NodeHandle(n, "pid")))return false;

    pid_controller.resize(12);
    for (int i = 0; i < 12; i++){
      //pid_controller[i].init(ros::NodeHandle(n, "pid"));
      pid_controller[i].initPid(600, 0, 10, 10000, -10000);   
      //pid_controller[i].initParam("pid");
    }


    //sub_command_topic = n.subscribe<std_msgs::Float64>("command", 1, &HyQ2maxJointsPositionController::setJointSetPointCB, this);
    sub_command_topic = n.subscribe<hyq2max_joints_position_controller::HyQ2max_command>("command", 1, &HyQ2maxJointsPositionController::setJointSetPointCB, this);

    //pub_state_topic = n.advertise<geometry_msgs::Pose>("hyq2max/test", 1000);
    pub_state_topic = n.advertise<hyq2max_joints_position_controller::HyQ2max_joints>("state_joint", 1000);

    return true;
  }

  
  void update(const ros::Time& time, const ros::Duration& period){
  
    //std::cout<< "UPDATE\n";
    for (int i = 0; i < 12; i++){
      position[i] = joints[i].getPosition();
      velocity[i] = joints[i].getVelocity();

      error[i] = position_ref[i] - position[i];
      error_dot[i] = velocity_ref[i] - velocity[i];

      commanded_effort[i] = pid_controller[i].computeCommand(error[i], error_dot[i], period);
      joints[i].setCommand(commanded_effort[i]);
    }

/*
    state = joint_.getPosition();
    state_dot = joint_.getVelocity();

    double error = position_ref - state;
    double error_dot = velocity_ref - state_dot;

    double commanded_effort = pid_controller.computeCommand(error, error_dot, period);
   
    joint_.setCommand(commanded_effort);
    */
    pub_joint_state(); 
    //std::cout<< "update\n";
    
  }


  //void setJointSetPointCB(const std_msgs::Float64ConstPtr& msg)
  void setJointSetPointCB(const hyq2max_joints_position_controller::HyQ2max_commandConstPtr  &msg)
  {
    
  	//position_ref = msg->data;
    for (int i = 0; i < 12; i++){
      position_ref[i] = msg->pos_command[i];
      velocity_ref[i] = msg->vel_command[i];
    }
  }


  void starting(const ros::Time& time) { 
    /*position_ref = joint_.getPosition();
    velocity_ref = joint_.getVelocity();

    double error = 0;//position_ref - state;
    double error_dot = 0;//velocity_ref - state_dot;

   // commanded_effort = pid_controller_.computeCommand(error, time);

    //joint_.setCommand(commanded_effort);

    pid_controller.reset();*/
    
    for (int i = 0; i < 12; i++){
      position_ref[i] = joints[i].getPosition();
      velocity_ref[i] =   joints[i].getVelocity();
      pid_controller[i].reset();
    }

  }


  void stopping(const ros::Time& time) { }


  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
  {
    for (int i = 0; i < 12; i++){
      pid_controller[i].setGains(p,i,d,i_max,i_min,antiwindup);	
    }
  }

  void pub_joint_state(){
    hyq2max_joints_position_controller::HyQ2max_joints pub_state;

    for (int i = 0; i < 12; i++){
      pub_state.joints_pos[i] = position[i];
      pub_state.joints_vel[i] = velocity[i];
      pub_state.joints_pos_ref[i] = position_ref[i];
      pub_state.joints_vel_ref[i] = velocity_ref[i];
      pub_state.joints_command[i] = commanded_effort[i];
    }
    
    pub_state_topic.publish(pub_state);
 
  }

private:
  //hardware_interface::JointHandle joints[12];
  std::vector<hardware_interface::JointHandle> joints;
  ros::Subscriber sub_command_topic;
  ros::Publisher pub_state_topic;
  //control_toolbox::Pid pid_controller[12];
  std::vector<control_toolbox::Pid> pid_controller;
  double position_ref[12];
  double velocity_ref[12];
  double position[12];
  double velocity[12];
  double error[12];
  double error_dot[12];
  double commanded_effort[12];


};
PLUGINLIB_DECLARE_CLASS(hyq2max_joints_position_controller, PositionController, hyq2max_joints_position_controller_ns::HyQ2maxJointsPositionController, controller_interface::ControllerBase);

}//namespace