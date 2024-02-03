#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Float64MultiArray.h>

namespace template_namespace {
    class Template : public controller_interface::Controller<hardware_interface::VelocityJointInterface>{
        public:
            bool init(hardware_interface::VelocityJointInterface* hardware, ros::NodeHandle &node_handler){

            }
            void update(const ros::Time& time, const ros::Duration& period){

            }
            void starting(const ros::Time& time){}
            void stopping(const ros::Time& time){}
        private:
            float hi;
    };
    PLUGINLIB_EXPORT_CLASS(template_namespace::Template, controller_interface::ControllerBase);
}
