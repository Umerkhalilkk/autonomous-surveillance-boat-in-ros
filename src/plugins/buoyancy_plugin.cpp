#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class SimpleBuoyancy : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->link = this->model->GetLink("link");

      // Connect to the update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SimpleBuoyancy::OnUpdate, this));
      
      gzmsg << "Buoyancy Plugin Loaded for " << this->model->GetName() << std::endl;
    }

    public: void OnUpdate()
    {
      // Get current pose
      ignition::math::Pose3d pose = this->model->WorldPose();
      double z = pose.Pos().Z();

      // Water level is at 0.0
      double water_level = 0.0;

      // Simple physics: If below water, apply upward force
      if (z < water_level)
      {
        // Archimedes approximation + Damping to stop infinite bouncing
        // Force = (Depth * BuoyancyFactor) - (Velocity * Damping)
        
        double depth = water_level - z;
        double buoyancy_force = depth * 50.0; // Tune this number based on mass
        
        // Get vertical velocity for damping
        ignition::math::Vector3d lin_vel = this->link->WorldLinearVel();
        double damping = lin_vel.Z() * 2.0; 

        double final_force = buoyancy_force - damping;

        // Apply force to the center of mass
        this->link->AddForce(ignition::math::Vector3d(0, 0, final_force));
      }
    }

    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SimpleBuoyancy)
}
