#include "virtual_dc_motor_driver.hpp"

VirtualDCMotorDriverNode::VirtualDCMotorDriverNode() : loop_rate_(400),
                                           prevPos_(0),
                                           currPos_(0)
{
    nh_ = ros::NodeHandle("virtual_dc_motor_driver");
    get_position_sub_ = nh_.subscribe("/virtual_dc_motor/get_position", 10, &VirtualDCMotorDriverNode::getPositionCallback, this);
    get_velocity_pub_ = nh_.advertise<std_msgs::Float32>("/virtual_dc_motor_driver/get_velocity", 10);
}

void VirtualDCMotorDriverNode::getPositionCallback(const std_msgs::UInt16::ConstPtr &msg)
{
  prevPos_ = currPos_;
  currPos_ = msg->data;
  
  prevTimestamp_ = currTimestamp_;
  currTimestamp_ = std::chrono::high_resolution_clock::now();
  
  velocity_ = calculateVelocity();
}

float VirtualDCMotorDriverNode::calculateVelocity()
{
    int16_t posDiff = currPos_ - prevPos_;
    std::chrono::duration<float> timeDiff = currTimestamp_ - prevTimestamp_;
    
    float ratio = posDiff / timeDiff.count();
    
    return ratio * 60 / 4096;
}

void VirtualDCMotorDriverNode::run()
{
    int count = 0;
    while (ros::ok())
    {
        if (count % 4 == 0)
        {
            std_msgs::Float32 velocity_msg;
    	    velocity_msg.data = velocity_;
    	    get_velocity_pub_.publish(velocity_msg);
    	}
    	
    	count++;
        ros::spinOnce();
        loop_rate_.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_dc_motor_driver_node");
    VirtualDCMotorDriverNode node;
    node.run();
    return 0;
}
