#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace boxxo_controllers {
    class BoxxoArm : public controller_interface::Controller<hardware_interface::PositionJointInterface>{
        public:
            bool init(hardware_interface::PositionJointInterface* hardware, ros::NodeHandle &node_handler){
		ROS_INFO("FAT BALLS");
                std::string joint1;
                std::string joint2;
                std::string joint3;
                std::string joint4;
                std::string joint5;


                if(!node_handler.getParam("joint1", joint1)){
                    ROS_ERROR("joint1 missing");
                    return false;
                }
                if(!node_handler.getParam("joint2", joint2)){
                    ROS_ERROR("joint1 missing");
                    return false;
                }
                if(!node_handler.getParam("joint3", joint3)){
                    ROS_ERROR("joint1 missing");
                    return false;
                }
                if(!node_handler.getParam("joint4", joint4)){
                    ROS_ERROR("joint1 missing");
                    return false;
                }
                if(!node_handler.getParam("joint5", joint5)){
                    ROS_ERROR("joint1 missing");
                    return false;
                }

                j1 = hardware->getHandle(joint1);
                j2 = hardware->getHandle(joint2);
                j3 = hardware->getHandle(joint3);
                j4 = hardware->getHandle(joint4);
                j5 = hardware->getHandle(joint5);

                input = node_handler.subscribe<std_msgs::Float64MultiArray>("cmd", 1, &BoxxoArm::change_pos, this);
                input2 = node_handler.subscribe<std_msgs::Float64MultiArray>("cmd2", 1, &BoxxoArm::change_faster, this);

                return true;
            }

            void change_faster(const std_msgs::Float64MultiArray::ConstPtr& array){
                position_mutex.lock();
                j1_pos = array->data[0] * 2;
                j2_pos = array->data[1] * 2;
                j3_pos = array->data[2] * 2;
                j4_pos = array->data[3] * 2;
                j5_pos = array->data[4] * 2;
                position_mutex.unlock();
            }

            void change_pos(const std_msgs::Float64MultiArray::ConstPtr& array){
                // int i = 0;
                // double temp[5];
                // for(std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); it += 1){
                //     temp[i] = *it;
                //     i += 1;
                // }
                // j1_pos = temp[0];
                // j2_pos = temp[1];
                // j3_pos = temp[2];
                // j4_pos = temp[3];
                // j5_pos = temp[4];
                position_mutex.lock();
                j1_pos = array->data[0];
                j2_pos = array->data[1];
                j3_pos = array->data[2];
                j4_pos = array->data[3];
                j5_pos = array->data[4];
                position_mutex.unlock();
            }

            void update(const ros::Time& time, const ros::Duration& period){
                j1.setCommand(j1_pos);
                j2.setCommand(j2_pos);
                j3.setCommand(j3_pos);
                j4.setCommand(j4_pos);
                j5.setCommand(j5_pos);
            }
            void starting(const ros::Time& time){
                j1_pos = 0.0;
                j2_pos = 0.0;
                j3_pos = 0.0;
                j4_pos = 0.0;
                j5_pos = 0.0;
            }
            void stopping(const ros::Time& time){}

        private:
            ros::Subscriber input;
            ros::Subscriber input2;

            std::mutex position_mutex;

            double j1_pos;
            double j2_pos;
            double j3_pos;
            double j4_pos;
            double j5_pos;

            hardware_interface::JointHandle j1;
            hardware_interface::JointHandle j2;
            hardware_interface::JointHandle j3;
            hardware_interface::JointHandle j4;
            hardware_interface::JointHandle j5;
    };
    PLUGINLIB_EXPORT_CLASS(boxxo_controllers::BoxxoArm, controller_interface::ControllerBase);
}
