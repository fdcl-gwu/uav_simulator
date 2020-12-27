#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "fdcl/common_types.hpp"
#include "fdcl/ros_utils.hpp"
#include "fdcl/matrix_utils.hpp"


namespace igm = ignition::math;


namespace gazebo
{

class UavControlPlugin : public ModelPlugin
{

public: 
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->world = _model->GetWorld();
        this->model = _model;

        this->link_name = _sdf->GetElement("bodyName")->Get<std::string>();
        this->link = _model->GetLink(this->link_name);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection = event::Events::ConnectWorldUpdateBegin( \
            std::bind(&UavControlPlugin::update, this));

        // Start the ROS subscriber.
        this->topic_name = _sdf->GetElement("topicName")->Get<std::string>();
        this->sub_fm = this->n.subscribe(this->topic_name, 1, \
            UavControlPlugin::update_fm);
    }


    void update(void)
    {
        no_msg_counter++;
        if (no_msg_counter > 100)
        {
            this->reset_uav();

            if (!print_reset_message)
            {
                std::cout << ros::Time::now()
                    << ": no new force messages, resetting UAV .." 
                    << std::endl;
                print_reset_message = true;
            }
            return;
        }

        // Both must be in world frame.
        this->calculate_force();
        this->link->SetForce(this->force);
        this->link->SetTorque(this->M_out);
    }


    void calculate_force(void)
    {
        this->update_uav_rotation();

        fdcl::Vector3 force_body;
        this->ignition_to_eigen(this->f, force_body);

        fdcl::Vector3 force_world = this->R * force_body;
        this->eigen_to_ignition(force_world, this->force);

        fdcl::Vector3 M_body;
        this->ignition_to_eigen(this->M, M_body);

        fdcl::Vector3 M_world = this->R * M_body;
        this->eigen_to_ignition(M_world, this->M_out);
    }


    void update_uav_rotation(void)
    {
        ignition::math::Pose3d pose = this->link->WorldPose();
        ignition::math::Quaterniond q = pose.Rot();

        fdcl::Vector3 q13(q.X(), q.Y(), q.Z());
        double q4 = q.W();

        fdcl::Matrix3 hat_q = fdcl::hat(q13);
        this->R = eye3 + 2 * q4 * hat_q + 2 * hat_q * hat_q;
    }


    static void update_fm(const geometry_msgs::Wrench::ConstPtr& msg)
    {

        f[0] = msg->force.x;
        f[1] = msg->force.y;
        f[2] = msg->force.z;

        M[0] = msg->torque.x;
        M[1] = msg->torque.y;
        M[2] = msg->torque.z;

        no_msg_counter = 0;
        print_reset_message = false;
    }


    void reset_uav(void)
    {
        this->link->SetForce(zero_fM);
        this->link->SetTorque(zero_fM);
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
    ros::Time t0 = ros::Time::now();

    physics::ModelPtr model;
    physics::WorldPtr world;
    physics::LinkPtr link;

    std::string link_name;
    std::string topic_name;
    
    event::ConnectionPtr update_connection;
    ros::Subscriber sub_fm; 
    ros::NodeHandle n;

    static igm::Vector3d M;
    igm::Vector3d M_out;
    static igm::Vector3d f;

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

