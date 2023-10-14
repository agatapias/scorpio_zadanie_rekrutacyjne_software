#ifndef VIRTUAL_DC_MOTOR_DRIVER_HPP
#define VIRTUAL_DC_MOTOR_DRIVER_HPP

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <chrono>

class VirtualDCMotorDriverNode
{
public:
    VirtualDCMotorDriverNode();
    
    /**
     * @brief Callback function invoked every time new motor position is received
     *
     * @param msg  Pointer to the received message
     */
    void getPositionCallback(const std_msgs::UInt16::ConstPtr &msg);
     
    /**
     * @brief Funcion used to calculate the current velocity of motor in RPM.
     * Uses class parameters prevPos_, currPos_, prevTimestamp_ and currTimestamp_ to
     * calculate the velocity.
     * Saves the velocity to velocity_.
     *
     * @param msg  Pointer to the received message
     */
    float calculateVelocity();

    /**
     * @brief Main program loop resposible for updating the dc motor angular velocity.
     *
     */
    void run();

private:
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    
    // Define subscribers
    ros::Subscriber get_position_sub_;
    
    // Define publishers
    ros::Publisher get_velocity_pub_;
    
    // The previous position of the motor
    int16_t prevPos_;
    
    // The current position of the motor
    int16_t currPos_;
    
    // The timestamp when the previous motor position was received
    std::chrono::high_resolution_clock::time_point prevTimestamp_{};
    
    // The timestamp when the current motor position was received
    std::chrono::high_resolution_clock::time_point currTimestamp_{};
    
    // The current velocity of the motor
    float velocity_;
};

#endif
