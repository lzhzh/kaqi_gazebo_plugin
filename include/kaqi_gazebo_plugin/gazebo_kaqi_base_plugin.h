#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <sstream>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose2D.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/algorithm/string.hpp>

namespace gazebo
{
  class GazeboKaqiBasePlugin : public ModelPlugin
  {
      enum OdomSource
      {
          ENCODER = 0,
          WORLD = 1,
      };

  public:
      GazeboKaqiBasePlugin();
      virtual ~GazeboKaqiBasePlugin();

      virtual void Reset();

      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();

  private:
      void getParameterBoolean(sdf::ElementPtr _sdf , bool &_value,
                               const std::string _tag_name, bool default_value);
      void getWheelVelocities();
      void updateOdometryEncoder();
      void publishOdometry(double step_time);
      void publishWheelJointState();
      void cmdVelCallback( const geometry_msgs::TwistConstPtr &msg);

  private:


      physics::WorldPtr my_world_;
      physics::ModelPtr my_parent_;

      /// Parameters
      std::string node_namespace_;
      std::string left_wheel_joint_name_;
      std::string right_wheel_joint_name_;
      std::string caster_rotate_joint_name_;
      std::string caster_wheel_joint_name_;
      std::string odom_frame_;
      std::string base_frame_;
      std::string command_topic_;
      std::string odometry_topic_;
      double wheel_diameter_;
      double wheel_separation_;
      double wheel_torque_;
      double wheel_acceleration_;

      // ROS STUFF
      ros::NodeHandle *rosnode_;
      ros::Publisher odometry_publisher_;
      ros::Publisher diagnostic_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      ros::Publisher joint_state_publisher_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;
      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      /// Speeds of the wheels
      boost::mutex lock_;
      double linear_velocity_;
      double angular_velocity_;
      double *wheel_speed_;
      double wheel_speed_instr_[2];

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

      OdomSource odom_source_;
      geometry_msgs::Pose2D pose_encoder_;
      common::Time last_odom_update_;

      // joints
      std::vector<physics::JointPtr> joints_;

      // Flags
      bool publish_odom_tf_;
      bool publish_wheel_tf_;
      bool publish_wheel_joint_state_;


//      event::ConnectionPtr contact_event_;
      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;
  };
}
