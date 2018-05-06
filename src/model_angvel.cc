#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
  class ModelAngVel : public ModelPlugin
  {
    public: double update_rate;
    public: common::Time update_period,last_update_time;
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelAngVel::OnUpdate, this));
      this->update_rate = 20.0;
      this->update_period = common::Time(0, common::Time::SecToNano(1.0/this->update_rate));
      this->last_update_time = common::Time::GetWallTime();
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      if (common::Time::GetWallTime() - this->last_update_time < this->update_period)
        return;
      joint_l = this->model->GetJoint("robot_wheel_left_joint");
      joint_r = this->model->GetJoint("robot_wheel_right_joint");
      double vel_l = this->joint_l->GetVelocity(0);
      double vel_r = this->joint_r->GetVelocity(0);
      printf("wheel_l:%.3f	wheel_r:%.3f\n",vel_l,vel_r);
//      ROS_INFO_NAMED("left wheel speed: %.5f, right wheel speed: %.5f",vel_l,vel_r);
    }

    // Pointer to the model
    private: physics::ModelPtr model;
             physics::JointPtr joint_l,joint_r;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelAngVel)
}
