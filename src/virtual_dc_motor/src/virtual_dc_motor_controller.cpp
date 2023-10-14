#include "virtual_dc_motor_controller.hpp"

VirtualDCMotorControlNode::VirtualDCMotorControlNode() : loop_rate_(400),
                                           goalVelocity_(0),
                                           currVelocity_(0),
                                           cs_(0)
{
    nh_ = ros::NodeHandle("virtual_dc_motor_controller");
    get_velocity_sub_ = nh_.subscribe("/virtual_dc_motor_driver/get_velocity", 10, &VirtualDCMotorControlNode::getVelocityCallback, this);
    set_velocity_goal_sub_ = nh_.subscribe("/virtual_dc_motor_controller/set_velocity_goal", 10, &VirtualDCMotorControlNode::setVelocityGoalCallback, this);
    set_cs_pub_ = nh_.advertise<std_msgs::Int8>("/virtual_dc_motor/set_cs", 10);
}

void VirtualDCMotorControlNode::setVelocityGoalCallback(const std_msgs::Float32::ConstPtr &msg)
{
    goalVelocity_ = msg->data;
}

void VirtualDCMotorControlNode::getVelocityCallback(const std_msgs::Float32::ConstPtr &msg)
{
    currVelocity_ = msg->data;
}

int8_t VirtualDCMotorControlNode::getCS()
{   
    if (currVelocity_ < goalVelocity_ && cs_ < 99) {
    	cs_ = cs_ + 1;
    } else if (currVelocity_ > goalVelocity_ && cs_ > -99) {
    	cs_ = cs_ - 1;
    }
    
    return cs_;
}

void VirtualDCMotorControlNode::run()
{
    int count = 0;
    while (ros::ok())
    {
        if (count % 4 == 0)
        {
    	    std_msgs::Int8 cs_msg;
    	    cs_msg.data = getCS();
    	    set_cs_pub_.publish(cs_msg);
    	}
        count++;
        ros::spinOnce();
        loop_rate_.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_dc_motor_controller_node");
    VirtualDCMotorControlNode node;
    node.run();
    return 0;
}
