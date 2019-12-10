#include <urdf/model.h>
#include "rclcpp/rclcpp.hpp"

#include <memory>
// #include <boost.hpp>

class Parser : public rclcpp::Node
{
public:
  Parser()
  : Node("parser"), joints_(), model_(new urdf::Model())
  {
    std::string urdf_file = "model.xml";

    if (!model_->initFile(urdf_file)){
      RCLCPP_WARN(this->get_logger(), "NAY");
    }
    else RCLCPP_INFO(this->get_logger(), "YAY");

    
    std::vector<std::shared_ptr<urdf::Link>> links;
    model_->getLinks(links);
    for (size_t i = 0; i < links.size(); i++) {
      if (links[i]->child_joints.size() != 0) {     
        joints_.push_back(links[i]->child_joints[0]);
        std::cout << links[i]->child_joints[0]->name << std::endl;
        }   
    }

}

private:
  std::shared_ptr<urdf::Model> model_;
  std::vector<std::shared_ptr<urdf::Joint>> joints_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Parser>());
  rclcpp::shutdown();
  return 0;
}
