#include "rover_config/robot.hpp"


using std::cout;
using std::endl;

Robot::Robot(std::string robot_file):
	robot_file_(robot_file),
	model_(new urdf::Model()),
	joints_(),
	links_(){	
	
	if (!model_->initFile(robot_file)){
		cout << "Failed to parse urdf file" << endl;
	}
	else {
		cout << "Successfully parsed urdf file" << endl;
	}
	
	// Get Links
	model_->getLinks(links_);

	// Get Joints that are connected to received links
	for (std::shared_ptr<urdf::Link> link : links_) {
        for (std::shared_ptr<urdf::Joint> child_joint : link->child_joints) {
			joints_.push_back(child_joint);
        }
	}
}

void Robot::getLinkNames(std::vector<std::string> &link_names) {
	for (std::shared_ptr<urdf::Link> link : links_) {
		link_names.push_back(link->name);
	}
}

void Robot::getJointNames(std::vector<std::string> &joint_names) {
	for (std::shared_ptr<urdf::Joint> joint : joints_) {
		joint_names.push_back(joint->name);
	}
}

void Robot::getLinkMasses(std::vector<std::string> &link, std::vector<double> &link_masses) {

	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links_.size(); j ++) {
			if (link[i] == links_[j]->name) {
				if (links_[j]->inertial != nullptr) {			
					link_masses.push_back(links_[j]->inertial->mass);
				}
				else{
					link_masses.push_back(0.0);
				}
			}
		}
	}
}

void Robot::getLinkInertias(std::vector<std::string> &link, std::vector<std::vector<double>> &inertias) {

	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links_.size(); j ++) {
			if (links_[j]->name == link[i]) {				
				if (links_[i]->inertial != nullptr) {					
					std::vector<double> inert;
					inert.push_back(links_[i]->inertial->ixx);
					inert.push_back(links_[i]->inertial->ixy);
					inert.push_back(links_[i]->inertial->ixz);
					inert.push_back(links_[i]->inertial->iyy);
					inert.push_back(links_[i]->inertial->iyz);
					inert.push_back(links_[i]->inertial->izz);
					inertias.push_back(inert);
				}
				else {
					std::vector<double> inert;
					inert.push_back(0.0);
				    inert.push_back(0.0);
					inert.push_back(0.0);
					inert.push_back(0.0);
					inert.push_back(0.0);
					inert.push_back(0.0);
					inertias.push_back(inert);
				}
			}
		}
	}
}

void Robot::getLinkDimension(std::vector<std::string> &link, std::vector<std::vector<double>> &dimension) {

	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links_.size(); j ++) {
			if (links_[j]->name == link[i]) {
				if (links_[j]->collision != nullptr) {
					std::vector<double> dim;
					std::shared_ptr<urdf::Box> box = std::static_pointer_cast<urdf::Box>(links_[j]->collision->geometry);
					dim.push_back(box->dim.x);
					dim.push_back(box->dim.y);
					dim.push_back(box->dim.z);
					dimension.push_back(dim);
				}
				else {
					std::vector<double> dim;
					dimension.push_back(dim);
				}
			}
		}
	}
}

void Robot::getLinkPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose) {

	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links_.size(); j ++) {
			if (links_[j]->name == link[i]) {
				if (links_[j]->collision != nullptr) {
					double roll, pitch, yaw;
					std::vector<double> pse; 
					pse.push_back(links_[j]->collision->origin.position.x);
					pse.push_back(links_[j]->collision->origin.position.y);
					pse.push_back(links_[j]->collision->origin.position.z);
					
					links_[j]->collision->origin.rotation.getRPY(roll, pitch, yaw);
					pse.push_back(roll);
					pse.push_back(pitch);
					pse.push_back(yaw);					
					
					pose.push_back(pse);
				}
				else {
					std::vector<double> pse; 
					pose.push_back(pse);
				}
			}
		}
	}
}

void Robot::getLinkInertialPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose) {

	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links_.size(); j ++) {
			if (links_[j]->name == link[i]) {
				if (links_[j]->inertial != nullptr) {
					double roll, pitch, yaw;
					std::vector<double> pse;
					pse.push_back(links_[i]->inertial->origin.position.x);
					pse.push_back(links_[i]->inertial->origin.position.y);
					pse.push_back(links_[i]->inertial->origin.position.z);
					
					links_[j]->inertial->origin.rotation.getRPY(roll, pitch, yaw);
					pse.push_back(roll);
					pse.push_back(pitch);
					pse.push_back(yaw);
					pose.push_back(pse);
				}
				else {
					std::vector<double> pse;
					for (size_t k = 0; k < 6; k++) {
						pse.push_back(0.0);
					}
					pose.push_back(pse);
				}
			}
		}
	}
}

void Robot::getJointType(std::vector<std::string> &joint, std::vector<int> &type) {
	for (size_t i = 0; i < joint.size(); i++) {
		for (size_t j = 0; j < joints_.size(); j++) {
			if (joint[i] == joints_[j]->name) {
				type.push_back(joints_[j]->type);
			}
		}
	}	
}

void Robot::getJointOrigin(std::vector<std::string> &joints, std::vector<std::vector<double>> &origins) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joints_.size(); j++) {
			if (joints[i] == joints_[j]->name) {
				std::vector<double> orig;
				orig.push_back(joints_[j]->parent_to_joint_origin_transform.position.x);
				orig.push_back(joints_[j]->parent_to_joint_origin_transform.position.y);
				orig.push_back(joints_[j]->parent_to_joint_origin_transform.position.z);
				
				double roll, pitch, yaw;
				joints_[j]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
				orig.push_back(roll);
				orig.push_back(pitch);
				orig.push_back(yaw);
				origins.push_back(orig);
			}
		}
	}
}

void Robot::getJointAxis(std::vector<std::string> &joints, std::vector<std::vector<double>> &axis) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joints_.size(); j++) {
			if (joints[i] == joints_[j]->name) {
				std::vector<double> ax;
				ax.push_back(joints_[j]->axis.x);
				ax.push_back(joints_[j]->axis.y);
				ax.push_back(joints_[j]->axis.z);
				axis.push_back(ax);
			}
		}
	}
}