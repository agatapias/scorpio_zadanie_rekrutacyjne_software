#ifndef VIRTUAL_DC_MOTOR_CONTROLLER_HPP
#define VIRTUAL_DC_MOTOR_CONTROLLER_HPP

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

class VirtualDCMotorControlNode
{
public:
    VirtualDCMotorControlNode();
    
    /**
     * @brief Callback function invoked every time new velocity goal is received
     *
     * @param msg  Pointer to the received message
     */
    void setVelocityGoalCallback(const std_msgs::Float32::ConstPtr &msg);
    
    /**
     * @brief Callback function invoked every time new current velocity of motor is received
     *
     * @param msg  Pointer to the received message
     */
    void getVelocityCallback(const std_msgs::Float32::ConstPtr &msg);
     
    /**
     * @brief Function used to calculate the new control signal based on the relation
     * of current velocity to the goal velocity.
     *
     */
    int8_t getCS();

    /**
     * @brief Main program loop resposible for sending the updated control signal.
     *
     */
    void run();

private:
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    
    // Define subscribers
    ros::Subscriber get_velocity_sub_;
    ros::Subscriber set_velocity_goal_sub_;
    
    // Define publishers
    ros::Publisher set_cs_pub_;
    
    // The goal velocity of the motor
    int16_t goalVelocity_;
    
    // The current velocity of the motor
    float currVelocity_;
    
    // Currently saved control signal
    int8_t cs_;
};

#endif
