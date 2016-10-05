//#include <kaqi_gazebo_plugin/gazebo_kaqi_base_plugin.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>

#include "../include/kaqi_gazebo_plugin/gazebo_kaqi_base_plugin.h"

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace gazebo
{

enum {LEFT_WHEEL= 0, RIGHT_WHEEL=1, CASTER_ROTATE=2, CASTER_WHEEL=3};

GazeboKaqiBasePlugin::GazeboKaqiBasePlugin()
{

    linear_velocity_ = 0;
    angular_velocity_ = 0;

    wheel_speed_ = new double[2];
    wheel_speed_[LEFT_WHEEL] = 0.0;
    wheel_speed_[RIGHT_WHEEL] = 0.0;

    joints_.resize( 4 );
    joints_[0].reset();
    joints_[1].reset();
    joints_[2].reset();
    joints_[3].reset();
}

void GazeboKaqiBasePlugin::Reset()
{
    last_update_time_ = this->my_world_->GetSimTime();
    pose_encoder_.x = 0;
    pose_encoder_.y = 0;
    pose_encoder_.theta = 0;
    linear_velocity_ = 0;
    angular_velocity_ = 0;

    joints_[LEFT_WHEEL]->SetParam ( "fmax", 0, wheel_torque_ );
    joints_[RIGHT_WHEEL]->SetParam ( "fmax", 0, wheel_torque_ );
}

//
GazeboKaqiBasePlugin::~GazeboKaqiBasePlugin()
{
    rosnode_->shutdown();
    callback_queue_thread_.join();
    delete [] wheel_speed_;
    delete rosnode_;
}

void GazeboKaqiBasePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    std::cout << std::endl << std::endl << "Plugin Load ..." << std::endl << std::endl;

    this->my_world_ = _parent->GetWorld();

    this->my_parent_ = _parent;
    if (!this->my_parent_)
    {
        ROS_FATAL("Gazebo_ROS_Create controller requires a Model as its parent");
        return;
    }

    this->node_namespace_ = "";
    if (_sdf->HasElement("node_namespace"))
        this->node_namespace_ = _sdf->GetElement("node_namespace")->Get<std::string>() + "/";

    update_rate_ = 0;
    if (_sdf->HasElement("update_rate"))
      update_rate_ = _sdf->GetElement("update_rate")->Get<double>();

    getParameterBoolean(_sdf, publish_odom_tf_, "publish_odom_tf", true);
    getParameterBoolean(_sdf, publish_wheel_tf_, "publish_wheel_tf", true);
    getParameterBoolean(_sdf, publish_wheel_joint_state_, "publish_wheel_joint_state", true);

    odom_frame_ = "odom";
    if (_sdf->HasElement("odom_frame"))
        odom_frame_ = _sdf->GetElement("odom_frame")->Get<std::string>();

    base_frame_ = "base_footprint";
    if (_sdf->HasElement("base_frame"))
        base_frame_ = _sdf->GetElement("base_frame")->Get<std::string>();

    left_wheel_joint_name_ = "left_wheel_joint";
    if (_sdf->HasElement("left_wheel_joint"))
        left_wheel_joint_name_ = _sdf->GetElement("left_wheel_joint")->Get<std::string>();

    right_wheel_joint_name_ = "right_wheel_joint";
    if (_sdf->HasElement("right_wheel_joint"))
        right_wheel_joint_name_ = _sdf->GetElement("right_wheel_joint")->Get<std::string>();

    caster_rotate_joint_name_ = "caster_rotate_joint";
    if (_sdf->HasElement("caster_rotate_joint"))
        caster_rotate_joint_name_ = _sdf->GetElement("caster_rotate_joint")->Get<std::string>();

    caster_wheel_joint_name_ = "caster_wheel_joint";
    if (_sdf->HasElement("caster_wheel_joint"))
        caster_wheel_joint_name_ = _sdf->GetElement("caster_wheel_joint")->Get<std::string>();

    wheel_diameter_ = 0.16;
    if (_sdf->HasElement("wheel_diameter"))
      wheel_diameter_ = _sdf->GetElement("wheel_diameter")->Get<double>();

    wheel_separation_ = 0.44;
    if (_sdf->HasElement("wheel_separation"))
      wheel_separation_ = _sdf->GetElement("wheel_separation")->Get<double>();

    wheel_torque_ = 10.0;
    if (_sdf->HasElement("wheel_torque"))
      wheel_torque_ = _sdf->GetElement("wheel_torque")->Get<double>();

    wheel_acceleration_ = 0;
    if (_sdf->HasElement("wheel_acceleration"))
      wheel_acceleration_ = _sdf->GetElement("wheel_acceleration")->Get<double>();

    command_topic_ = "/cmd_vel";
    if (_sdf->HasElement("command_topic"))
        command_topic_ = _sdf->GetElement("command_topic")->Get<std::string>();

    odometry_topic_ = "/odom";
    if (_sdf->HasElement("odometry_topic"))
        odometry_topic_ = _sdf->GetElement("odometry_topic")->Get<std::string>();

    odom_source_ = ENCODER;

    // Initialize update rate stuff
    if ( update_rate_ > 0.0 )
        update_period_ = 1.0 / update_rate_;
    else
        update_period_ = 50.0;


    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_kaqi_base", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    }

    rosnode_ = new ros::NodeHandle( node_namespace_ );
    rosnode_->setCallbackQueue( &queue_ );
    cmd_vel_subscriber_ = rosnode_->subscribe(command_topic_, 1, &GazeboKaqiBasePlugin::cmdVelCallback, this );
    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    diagnostic_publisher_ = rosnode_->advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
    joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState>("/joint_states", 1);

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // Initialize joints
    joints_.resize ( 4 );
    joints_[LEFT_WHEEL] = my_parent_->GetJoint(left_wheel_joint_name_);
    joints_[RIGHT_WHEEL] = my_parent_->GetJoint(right_wheel_joint_name_);
    joints_[LEFT_WHEEL]->SetParam ( "fmax", 0, wheel_torque_ );
    joints_[RIGHT_WHEEL]->SetParam ( "fmax", 0, wheel_torque_ );
    // Caster joints
    joints_[CASTER_ROTATE] = my_parent_->GetJoint(caster_rotate_joint_name_);
    joints_[CASTER_WHEEL] = my_parent_->GetJoint(caster_wheel_joint_name_);

    // Initialize velocity stuff
    wheel_speed_[LEFT_WHEEL] = 0;
    wheel_speed_[RIGHT_WHEEL] = 0;

    // Initialize velocity support stuff
    wheel_speed_instr_[LEFT_WHEEL] = 0;
    wheel_speed_instr_[RIGHT_WHEEL] = 0;

    last_update_time_ = this->my_world_->GetSimTime();
    pose_encoder_.x = 0;
    pose_encoder_.y = 0;
    pose_encoder_.theta = 0;
    linear_velocity_ = 0;
    angular_velocity_ = 0;

    joints_[LEFT_WHEEL]->SetParam ( "fmax", 0, wheel_torque_ );
    joints_[RIGHT_WHEEL]->SetParam ( "fmax", 0, wheel_torque_ );

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboKaqiBasePlugin::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboKaqiBasePlugin::UpdateChild, this ) );

    ROS_INFO_STREAM("Finish loading kaqi gazebo base plugin.");
}

void GazeboKaqiBasePlugin::QueueThread()
{
    static const double timeout = 0.01;

    while ( rosnode_->ok() )
    {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboKaqiBasePlugin::UpdateChild()
{
    for ( int i = 0; i < 2; i++ )
    {
      if ( fabs(wheel_torque_ -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 )
        joints_[i]->SetParam ( "fmax", 0, wheel_torque_ );
    }

    if ( odom_source_ == ENCODER )
        updateOdometryEncoder();

    common::Time current_time = this->my_world_->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ )
    {
        publishOdometry ( seconds_since_last_update );
        if ( publish_wheel_joint_state_ )
            publishWheelJointState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();

        double current_speed[2];

        current_speed[LEFT_WHEEL] = joints_[LEFT_WHEEL]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[RIGHT_WHEEL] = joints_[RIGHT_WHEEL]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );

        if ( wheel_acceleration_ == 0 ||
                ( fabs ( wheel_speed_[LEFT_WHEEL] - current_speed[LEFT_WHEEL] ) < 0.01 ) ||
                ( fabs ( wheel_speed_[RIGHT_WHEEL] - current_speed[RIGHT_WHEEL] ) < 0.01 ) )
        {
            //if max_accel == 0, or target speed is reached
            joints_[LEFT_WHEEL]->SetParam ( "vel", 0, wheel_speed_[LEFT_WHEEL]/ ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT_WHEEL]->SetParam ( "vel", 0, wheel_speed_[RIGHT_WHEEL]/ ( wheel_diameter_ / 2.0 ) );
        }
        else
        {
            if ( wheel_speed_[LEFT_WHEEL]>=current_speed[LEFT_WHEEL] )
                wheel_speed_instr_[LEFT_WHEEL]+=fmin ( wheel_speed_[LEFT_WHEEL]-current_speed[LEFT_WHEEL],  wheel_acceleration_ * seconds_since_last_update );
            else
                wheel_speed_instr_[LEFT_WHEEL]+=fmax ( wheel_speed_[LEFT_WHEEL]-current_speed[LEFT_WHEEL], -wheel_acceleration_ * seconds_since_last_update );

            if ( wheel_speed_[RIGHT_WHEEL]>current_speed[RIGHT_WHEEL] )
                wheel_speed_instr_[RIGHT_WHEEL]+=fmin ( wheel_speed_[RIGHT_WHEEL]-current_speed[RIGHT_WHEEL], wheel_acceleration_ * seconds_since_last_update );
            else
                wheel_speed_instr_[RIGHT_WHEEL]+=fmax ( wheel_speed_[RIGHT_WHEEL]-current_speed[RIGHT_WHEEL], -wheel_acceleration_ * seconds_since_last_update );

            // ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
            // ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);
            joints_[LEFT_WHEEL]->SetParam ( "vel", 0, wheel_speed_instr_[LEFT_WHEEL] / ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT_WHEEL]->SetParam ( "vel", 0, wheel_speed_instr_[RIGHT_WHEEL] / ( wheel_diameter_ / 2.0 ) );
        }
        last_update_time_+= common::Time ( update_period_ );
    }
}

void GazeboKaqiBasePlugin::cmdVelCallback( const geometry_msgs::TwistConstPtr &msg)
{
    boost::mutex::scoped_lock scoped_lock ( lock_ );

    last_cmd_vel_time_ = this->my_world_->GetSimTime();
    linear_velocity_ = msg->linear.x;
    angular_velocity_ = msg->angular.z;
}

void GazeboKaqiBasePlugin::getWheelVelocities()
{
    boost::mutex::scoped_lock scoped_lock ( lock_ );

    double vr = linear_velocity_;
    double va = angular_velocity_;

    wheel_speed_[LEFT_WHEEL] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_WHEEL] = vr + va * wheel_separation_ / 2.0;
}

void GazeboKaqiBasePlugin::updateOdometryEncoder()
{
    double vl = joints_[LEFT_WHEEL]->GetVelocity ( 0 );
    double vr = joints_[RIGHT_WHEEL]->GetVelocity ( 0 );
    common::Time current_time = this->my_world_->GetSimTime();
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double b = wheel_separation_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double theta = ( sr - sl ) / b;


    double dx = ( sl + sr ) /2.0 * cos ( pose_encoder_.theta + ( sr - sl ) / ( 2.0*b ) );
    double dy = ( sl + sr ) /2.0 * sin ( pose_encoder_.theta + ( sr - sl ) / ( 2.0*b ) );
    double dtheta = ( sr - sl ) /b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dx/seconds_since_last_update;
    odom_.twist.twist.linear.y = dy/seconds_since_last_update;
}

void GazeboKaqiBasePlugin::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER )
    {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );
    }
    if ( odom_source_ == WORLD )
    {
        // getting data form gazebo world
        math::Pose pose = this->my_parent_->GetWorldPose();
        qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
        vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        math::Vector3 linear;
        linear = this->my_parent_->GetWorldLinearVel();
        odom_.twist.twist.angular.z = this->my_parent_->GetWorldAngularVel().z;

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.rot.GetYaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.y - sinf ( yaw ) * linear.x;
    }

    if( publish_odom_tf_ )
    {
        tf::Transform base_footprint_to_odom ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( base_footprint_to_odom, current_time,
                                   odom_frame_, base_frame_ ) );
    }

    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame_;
    odom_.child_frame_id = base_frame_;

    odometry_publisher_.publish ( odom_ );
}

void GazeboKaqiBasePlugin::publishWheelJointState()
{
    ros::Time current_time = ros::Time::now();

    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = current_time;
    joint_states.name.resize ( joints_.size() );
    joint_states.position.resize ( joints_.size() );

//    ROS_INFO("Publish joint state %d", joints_.size());

    //
    diagnostic_msgs::DiagnosticArray diag;
    diag.header.stamp = current_time;

    for ( int i = 0; i < joints_.size(); i++ ) {
        physics::JointPtr joint = joints_[i];
        math::Angle angle = joint->GetAngle ( 0 );
        joint_states.name[i] = joint->GetName();
        joint_states.position[i] = angle.Radian () ;
        //
        std::stringstream ss;
        diagnostic_msgs::DiagnosticStatus status;
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.name = "Joint ("+joint->GetName()+")";
        status.message = "OK";
        status.values.resize(1);
        status.values[0].key = "Position";
        ss << angle.Radian();
        status.values[0].value = ss.str();
        //
        diag.status.push_back(status);
    }
    joint_state_publisher_.publish ( joint_states );
    diagnostic_publisher_.publish( diag );
}

void GazeboKaqiBasePlugin::getParameterBoolean(sdf::ElementPtr _sdf , bool &_value,
                                                  const std::string _tag_name, bool default_value)
{
    if (_sdf->HasElement( _tag_name ) )
    {
        std::string value = _sdf->GetElement( _tag_name )->Get<std::string>();
//        if(boost::iequals(value, std::string("true")))
        if( !value.compare("true"))
        {
            _value = true;
        }
//        else if(boost::iequals(value, std::string("false")))
        else if( !value.compare("false"))
        {
            _value = false;
        }
        else
        {
            _value = default_value;
        }
    }
    else
    {
        _value = default_value;
    }
}

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN( GazeboKaqiBasePlugin )
}
