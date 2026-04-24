#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>

using namespace std::chrono_literals;

class JointStatesParserNode : public rclcpp::Node
{
public:
    JointStatesParserNode() : Node("joint_states_parser_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing joint_states_parser_node...");

        canonical_joints_ = {
			// Virtual joint
			"virtual_joint",
			// Torso
			"torso_joint",
			// Right arm
			"rf_shoulder_joint", "rs_shoulder_joint", "r_arm_twist_joint",
			"r_elbow_joint", "r_wrist_joint", "r_hand_joint",
			"right_gripper_L_joint", "right_gripper_R_joint",
			// Left arm
			"lf_shoulder_joint", "ls_shoulder_joint", "l_arm_twist_joint",
			"l_elbow_joint", "l_wrist_joint", "l_hand_joint",
			"left_gripper_L_joint", "left_gripper_R_joint",
			// Head
			"head_pan_joint", "head_tilt_joint",
			// Base wheels
			"wheel0_joint", "wheel1_joint", "wheel2_joint",
		};

        for (const auto& name : canonical_joints_)
            joint_positions_[name] = 0.0;

        expected_joints_ = {
            "torso_joint",
            "rf_shoulder_joint", "rs_shoulder_joint", "r_arm_twist_joint",
            "r_elbow_joint", "r_wrist_joint", "r_hand_joint",
            "lf_shoulder_joint", "ls_shoulder_joint", "l_arm_twist_joint",
            "l_elbow_joint", "l_wrist_joint", "l_hand_joint",
			"head_pan_joint", "head_tilt_joint",
			"wheel0_joint", "wheel1_joint", "wheel2_joint",
        };

        for (const auto& name : expected_joints_)
            received_[name] = false;

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hardware/joint_states",
            10,
            std::bind(&JointStatesParserNode::joint_state_callback, this,
                      std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/manipulation/joint_states", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&JointStatesParserNode::publish_callback, this));

        RCLCPP_INFO(this->get_logger(), "Joint states parser ready");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> canonical_joints_;
    std::vector<std::string> expected_joints_;

    std::unordered_map<std::string, double> joint_positions_;
    std::unordered_map<std::string, bool> received_;
    bool all_received_ = false;

    std::mutex mutex_;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const auto& name = msg->name[i];
            if (joint_positions_.count(name) && i < msg->position.size())
            {
                joint_positions_[name] = msg->position[i];
                received_[name] = true;
            }
        }

        if (!all_received_)
        {
            bool all = true;
            for (const auto& name : expected_joints_)
            {
                if (!received_[name])
                {
                    all = false;
                    break;
                }
            }
            if (all)
            {
                all_received_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "All joints received — publishing to /manipulation/joint_states");
            }
        }
    }

    void publish_callback()
	{
		std::lock_guard<std::mutex> lock(mutex_);

		sensor_msgs::msg::JointState msg;
		msg.header.stamp = this->now();
		msg.header.frame_id = "";

		for (const auto& name : canonical_joints_)
		{
		    msg.name.push_back(name);

		    if (received_.count(name) && received_[name])
		        msg.position.push_back(joint_positions_.at(name));
		    else
		        msg.position.push_back(0.0);
		}

		pub_->publish(msg);
	}
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatesParserNode>());
    rclcpp::shutdown();
    return 0;
}
