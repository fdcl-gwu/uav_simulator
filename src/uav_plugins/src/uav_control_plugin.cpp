// #include <functional>
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>

// #include <boost/thread.hpp>
// #include <boost/thread/mutex.hpp>
// // #include <ros/callback_queue.h>
// // #include <ros/subscribe_options.h>

#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>

// #include <ros/ros.h>
// #include "std_msgs/String.h"

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>


#include "fdcl/common_types.hpp"
#include "fdcl/ros_utils.hpp"
#include "fdcl/matrix_utils.hpp"


namespace igm = ignition::math;


namespace gazebo
{

class UavControlPlugin : public ModelPlugin
{

public: 
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        std::cout << "Loading UAV Control Plugin\n";

        rn_ = gazebo_ros::Node::Get(sdf);

        world_ = model->GetWorld();
        model_ = model;

        link_name_ = sdf->GetElement("body_name")->Get<std::string>();
        link_ = model->GetLink(link_name_);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin( \
            std::bind(&UavControlPlugin::update, this));

        // Start the ROS subscriber.
        topic_name_ = sdf->GetElement("topic_name")->Get<std::string>();

        sub_fm_ = rn_->create_subscription<geometry_msgs::msg::Wrench>( \
            topic_name_, 1, std::bind(&UavControlPlugin::update_fm, this, \
            std::placeholders::_1));
    }


    void update(void)
    {
        no_msg_counter++;
        if (no_msg_counter > 100)
        {
            reset_uav();

            if (!print_reset_message)
            {
                std::cout << "UAV Plugin: No new force messages"
                    << "\tresetting UAV\n";
                print_reset_message = true;
            }
            return;
        }

        // Both must be in world frame.
        calculate_force();
        link_->SetForce(force);
        link_->SetTorque(M_out);
    }


    void calculate_force(void)
    {
        update_uav_rotation();

        fdcl::Vector3 force_body;
        ignition_to_eigen(this->f, force_body);

        fdcl::Vector3 force_world = this->R * force_body;
        eigen_to_ignition(force_world, this->force);

        fdcl::Vector3 M_body;
        ignition_to_eigen(this->M, M_body);

        fdcl::Vector3 M_world = R * M_body;
        eigen_to_ignition(M_world, this->M_out);
    }


    void update_uav_rotation(void)
    {
        ignition::math::Pose3d pose = link_->WorldPose();
        ignition::math::Quaterniond q = pose.Rot();

        fdcl::Vector3 q13(q.X(), q.Y(), q.Z());
        double q4 = q.W();

        fdcl::Matrix3 hat_q = fdcl::hat(q13);
        R = eye3 + 2 * q4 * hat_q + 2 * hat_q * hat_q;
    }


    void update_fm(const geometry_msgs::msg::Wrench &msg) const
    {
        f[0] = msg.force.x;
        f[1] = msg.force.y;
        f[2] = msg.force.z;

        M[0] = msg.torque.x;
        M[1] = msg.torque.y;
        M[2] = msg.torque.z;

        no_msg_counter = 0;
        print_reset_message = false;
    }


    void reset_uav(void)
    {
        link_->SetForce(zero_fM);
        link_->SetTorque(zero_fM);
    }


    void ignition_to_eigen(
        const ignition::math::Vector3d input, fdcl::Vector3 &output
    )
    {
        output(0) = input[0];
        output(1) = input[1];
        output(2) = input[2];
    } 


    void eigen_to_ignition(
        const fdcl::Vector3 input, ignition::math::Vector3d &output
    )
    {
        output[0] = input(0);
        output[1] = input(1);
        output[2] = input(2);
    }


private:
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::LinkPtr link_;

    std::string link_name_;
    std::string topic_name_;
    
    event::ConnectionPtr update_connection_;

    rclcpp::Node::SharedPtr rn_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_fm_;


    static igm::Vector3d f;
    static igm::Vector3d M;
    igm::Vector3d M_out;

    fdcl::Matrix3 R = fdcl::Matrix3::Identity();
    igm::Vector3d force = igm::Vector3d::Zero;

    static int no_msg_counter;
    static bool print_reset_message;
    
    const igm::Vector3d zero_fM = igm::Vector3d::Zero;
    const fdcl::Matrix3 eye3 = fdcl::Matrix3::Identity();
};


// Register this plugin with the simulator.
GZ_REGISTER_MODEL_PLUGIN(UavControlPlugin)


}  // End of namespace gazebo.

igm::Vector3d gazebo::UavControlPlugin::f = igm::Vector3d::Zero;
igm::Vector3d gazebo::UavControlPlugin::M = igm::Vector3d::Zero;
int gazebo::UavControlPlugin::no_msg_counter = 0;
bool gazebo::UavControlPlugin::print_reset_message = false;

