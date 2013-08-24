#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <stdio.h>

namespace gazebo
{
  class AddForce : public ModelPlugin
  {

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // read option args in sdf tags
      this->link_name = "root";
      if (_sdf->HasElement("linkname")) {
	this->link_name = _sdf->Get<std::string>("linkname");
      }
      this->force = math::Vector3(0, 0, 0);
      if (_sdf->HasElement("force")) {
	this->force = _sdf->Get<math::Vector3>("force");
      }
      this->torque = math::Vector3(0, 0, 0);
      if (_sdf->HasElement("torque")) {
	this->torque = _sdf->Get<math::Vector3>("torque");
      }

      // find root link
      this->link = this->model->GetLink(this->link_name);
      if(!this->link) {
	gzerr << "Root link are not found. (link_name is "<< this->link_name << ")" << std::endl;
	return;
      }
      
      // Listen to the update event.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AddForce::OnUpdate, this, _1));

      gzmsg << "AddForcePlugin was loaded ! ( force: " << this->force << "  torque: " << this->torque << " )" << std::endl;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->link->AddForce(this->force);
      this->link->AddTorque(this->torque);
    }

  private:
    physics::ModelPtr model;
    std::string link_name;
    math::Vector3 force;
    math::Vector3 torque;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;

  };

  GZ_REGISTER_MODEL_PLUGIN(AddForce)
}
