/*********************************************************************************
    combined_resource_controller.cpp

    Example file for a controller plugin.

    The controllers are declared as a class inside a namespace stating the family of controllers.
    This class mus be a son of the controller_interface/controller_base class. The most important
    functions of the code below are:

    initRequest: This function contains the code to be executed at the moment the controller
                 manager calls the service to load the plugin. This function is executed only one
                 time before runing the controller's loop code. In this fucntion all variables
                 must be initialized, ros parameters read and all the objects constructed.

    update: This function contains the code to be executed every loop. The control law should
            be implemented here.

    stopping: Code to be executed one time when the controller is going to be unloaded. Here must
              be called all the destructors for the object and closed any port or comunication used
              on the controller.

**********************************************************************************/

#include <hardware_interface/internal/demangle_symbol.h>    // Hardware interface headers
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <controller_interface/controller_base.h>           // Controller class base structure
#include <pluginlib/class_list_macros.h>                    // Header with tools to build plugins

using namespace hardware_interface;
using namespace std;

namespace controllers_tutorials{

class CombinedResourceController : public controller_interface::ControllerBase
{
public:

  bool initRequest(hardware_interface::RobotHW* robot_hw,
                   ros::NodeHandle&             root_nh,
                   ros::NodeHandle &            controller_nh,
                   ClaimedResources&       claimed_resources)
  {
    // Check if construction finished cleanly
    if (state_ != ControllerState::CONSTRUCTED)
    {
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      return false;
    }

    // Get a pointer to the joint position control interface
    PositionJointInterface* pos_iface = robot_hw->get<PositionJointInterface>();
    if (!pos_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                getHardwareInterfaceType().c_str());
      return false;
    }

    // Return which resources are claimed by this controller
    pos_iface->clearClaims();
    if (!init(pos_iface,
              root_nh,
              controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      std::cerr  << "FAILED LOADING WALKING" << std::endl;
      return false;
    }
    //claimed_resources = pos_iface->getClaims();

    claimed_resources.push_back(InterfaceResources(internal::demangledTypeName<PositionJointInterface>(), pos_iface->getClaims()));
    pos_iface->clearClaims();

    // success
    state_ = ControllerState::INITIALIZED;
    return true;
  }

  bool init(PositionJointInterface*     pos_iface,
            ros::NodeHandle&            /*root_nh*/,
            ros::NodeHandle&            controller_nh)
  {
    // Hardware interfaces
    if (!initJoints(pos_iface, controller_nh) )
    {
      ROS_ERROR_STREAM("Failed to initialize controller '" << internal::demangledTypeName(*this) << "'");
      return false;
    }

    cont =0.0;
    return true;
  }

  bool initJoints(PositionJointInterface* pos_iface,
                  ros::NodeHandle&        controller_nh)
  {
    // Get joint names from the parameter server
    using namespace XmlRpc;
    XmlRpcValue joint_names;
    if (!controller_nh.getParam("joints", joint_names))
    {
      ROS_ERROR_STREAM("No joints given (namespace:" << controller_nh.getNamespace() << ").");
      return false;
    }
    if (joint_names.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR_STREAM("Malformed joint specification (namespace:" << controller_nh.getNamespace() << ").");
      return false;
    }

    // Populate temporary container of joint handles
    vector<hardware_interface::JointHandle> joints_tmp;
    for (int i = 0; i < joint_names.size(); ++i)
    {
      XmlRpcValue &name_value = joint_names[i];
      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Array of joint names should contain all strings (namespace:" << controller_nh.getNamespace() << ").");
        return false;
      }
      const string joint_name = static_cast<string>(name_value);

      // Get a joint handle
      try
      {
        joints_tmp.push_back(pos_iface->getHandle(joint_name));
        ROS_DEBUG_STREAM("Found joint '" << joint_name << "' in '" <<
                         getHardwareInterfaceType() << "'");
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in '" <<
                         getHardwareInterfaceType() << "'");
        return false;
      }
    }

    // Member list of joint handles is updated only once all resources have been claimed
    joints_ = joints_tmp;

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // This time division is because of an error in gazebo.urdf.xacro
    // The simulation loops at a 1ms rate but the real robot loops at 10ms
    // This is one solution to this issue and MUST BE COMENTED when deploying
    // on the real robot.
    // Other posible solution is to fix it at the file /gazebo/gazebo.urdf.xacro
    // in the tiago_robot package.
    // If you use fast dynamics, you need to adjust the time step to get 100 Hz update.

    if( (time.nsec % 10000000) == 0)   // Coment this line when working on the real robot
    {
      cont+=0.01;
      double pos = 0.0;     // For HSRB
      //double pos = -0.2;  // For Tiago

      joints_[2].setCommand(0.2*sin(cont) + pos);

      //ROS_WARN_STREAM("sin " << joints_[2].getPosition());
      //ROS_WARN_STREAM("time " << time.toSec() );
    }
    //ROS_INFO_STREAM("time " << time.toSec() );
  }

  void stopping(const ros::Time& time)
  {}

  std::string getHardwareInterfaceType() const {return hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>();}

private:
  // Definition of variable members for the controller class.
  double cont;

  // Define Hardware interface resources. This must match the stated on the config.yaml file.
  std::vector<hardware_interface::JointHandle> joints_;
};

// This declares the class as a plugin library, the namespace and the class name must match the above code.
// The last argument of the function must remain as it is because it defines it as a plugin for the controller manager.
PLUGINLIB_EXPORT_CLASS( controllers_tutorials::CombinedResourceController, controller_interface::ControllerBase);

}
