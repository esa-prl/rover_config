#include <urdf/model.h>
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include "rover_config/robot.hpp"

class Parser : public rclcpp::Node
{
public:
  Parser()
  : Node("parser"), joints_(), model_(new urdf::Model())
  {
      
    std::string urdf_file = "model.xml";
    // std::string urdf_file = "scalebot.xml";

    std::string mode = "AUTO";

    if (mode == "AUTO") {


      std::cout << "AUTO   READOUT:" << std::endl;
      std::cout << "###############" << std::endl;

      Robot robot (urdf_file);

      robot.getLinkNames(link_names_);
      robot.getJointNames(joint_names_);
      robot.getJointOrigin(joint_names_, origins_);


      for (std::string link_name : link_names_) {
        std::cout << "\t "<<  link_name << std::endl;
      }
      
      std::cout << "Found " << link_names_.size() << " Links" << std::endl;

      for (std::string joint_name : joint_names_) {
        std::cout << "\t "<<  joint_name << std::endl;
        // for (size_t j = 0; j < origins_[i].size(); j++){
        //   std::cout << origins_[i][j] << std::endl;
        // }

      }

      std::cout << "Found " << joint_names_.size() << " Joints" << std::endl;
    
      
      robot.getJointOrigin(joint_names_, origins_);

      for (size_t i = 0; i < joint_names_.size(); i++){
        std::cout << "\t "<<  joint_names_[i] << std::endl;

        for (double value : origins_[i])
        {
          std::cout << "\t \t"<< value << std::endl;      
        }

      }

    }
    else {
      

    std::cout << "MANUAL READOUT:" << std::endl;
    std::cout << "###############" << std::endl;

    if (!model_->initFile(urdf_file)){
      RCLCPP_WARN(this->get_logger(), "NAY");
    }
    else RCLCPP_INFO(this->get_logger(), "Successfully parsed urdf file");
    
    // 
    model_->getLinks(links_);

    for (size_t i = 0; i < links_.size(); i++) {
      std::cout << "\t "<< links_[i]->name << std::endl;
    }
    std::cout << "Found " << links_.size() << " Links" << std::endl;


    int joints_count = 0;

    for (size_t i = 0; i < links_.size(); i++) {
      if (links_[i]->child_joints.size() != 0) {
        for (std::shared_ptr<urdf::Joint> child_joint : links_[i]->child_joints) {
          joints_.push_back(child_joint);
          std::cout << "\t "<< child_joint->name << std::endl;
          joints_count++;
        }     
      }   
    }
    std::cout << "Found " << joints_count << " Joints" << std::endl;

    }

}

private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<std::vector<double>> origins_;

  std::shared_ptr<urdf::Model> model_;
  std::vector<std::shared_ptr<urdf::Joint>> joints_;
  std::vector<std::shared_ptr<urdf::Link>> links_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Parser>());
  rclcpp::shutdown();
  return 0;
}
