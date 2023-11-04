#include "intersection_checker/intersection_checker.hpp"

using namespace std::chrono_literals;

Intersection_Checker::Intersection_Checker()
: Node("intersection_checker"){

  car_id = this->declare_parameter<char>(
    "car_id",
    '0');

  auto ip_address_ = this->declare_parameter<std::string>(
    "ip_address",
    "localhost");

  // Get QoS profile 
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(
    qos_profile.history,
    qos_profile.depth
    ),
    qos_profile);

  target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
  publisher_ = this->create_publisher<std_msgs::msg::String>("/controller_selector", qos);
  intersection_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/intersection_check", qos);


  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Call on_timer function every second and a half -> For Behavior Tree
  timer_ = this->create_wall_timer(
    100ms, std::bind(&Intersection_Checker::check_position, this));
    
  mqtt::connect_options connOpts;
  connOpts.set_keep_alive_interval(20);
  connOpts.set_clean_session(true);

  ADDRESS = "tcp://" + ip_address_ + ":1883";

  std::string client_id = CLIENT_ID + car_id;
  cli_ = new mqtt::client(ADDRESS, client_id);
  cli_->connect(connOpts);
  cli_->subscribe("/vtl/response",1);

  this->bt_call(1);
}

Intersection_Checker::~Intersection_Checker()
{
  cli_->disconnect();
  delete cli_;
}


void Intersection_Checker::bt_call(bool free_intersection)
{
  RCLCPP_INFO(this->get_logger(), "Free_intersection for BT: '%s'", free_intersection ? "True" : "False");

  auto message = std_msgs::msg::Bool();
  message.data = free_intersection;
  intersection_publisher_->publish(message);

}


void Intersection_Checker::vtl_request(std::string intersection)
{
  //car has to stop in a situation where
  //a free intersection is requested
  this->bt_call(0);

  // MQTT
  std::string msg_string = car_id+intersection;
  auto msg_request = mqtt::message::create("/vtl/request", msg_string , 1, false);
  
  RCLCPP_DEBUG(this->get_logger(), "Publishing request to vtl: '%s'", msg_string.c_str());

  cli_->publish(msg_request);
  

  while (true){
    auto msg = cli_->consume_message();
    RCLCPP_DEBUG(this->get_logger(), "Message from vtl: '%s'", msg->to_string().c_str());
    if(msg->to_string().c_str()[0] == car_id){
      if (msg->to_string().c_str()[1] == '1')
      {
        RCLCPP_INFO(this->get_logger(), "Request to VTL successful & Intersection Free");
        //send the information of a free intersection to the BT
        this->bt_call(1);
        switch_controller = true;
        break;
      }
      else if (msg->to_string().c_str()[1] == '0')
      {
        RCLCPP_INFO(this->get_logger(), "Intersection occupied");
      }
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Received message from VTL not matching the car_id!");
    }
  }
}

void Intersection_Checker::vtl_free()
{
  std::string id(1, car_id); 
  auto msg_free = mqtt::message::create("/vtl/free", id , 1, false);
  
  RCLCPP_DEBUG(this->get_logger(), "Publishing free to vtl: '%d'", id[0]);
  cli_->publish(msg_free);
  new_intersection = true;

  RCLCPP_INFO(this->get_logger(), "Intersection released");
}


void Intersection_Checker::check_position()
{
  // Store frame names in variables that will be used to
  // compute transformations
  std::string fromFrameRel = target_frame_.c_str();
  std::string toFrameRel = "map";

  geometry_msgs::msg::TransformStamped t;

  auto message = std_msgs::msg::String();
  message.data = "FollowPath";

  try {
      t = tf_buffer_->lookupTransform(
          toFrameRel, fromFrameRel,
          tf2::TimePointZero);
    // Intersection 0
    // Intersection area
    if (t.transform.translation.x > 1.3 and t.transform.translation.x < 2.6 and t.transform.translation.y > -0.6 and t.transform.translation.y < 0.6) {
        RCLCPP_INFO(this->get_logger(), "Intersection Area 1 detected");
    }

  // Stop area right
    else if (t.transform.translation.x > 2.0 and t.transform.translation.x < 2.6 and t.transform.translation.y > -1.4 and t.transform.translation.y < -0.6) {
        RCLCPP_INFO(this->get_logger(), "Stop Area Right 1 detected");
        message.data = "FollowPath";
        if(new_intersection){
          vtl_request("0");
          new_intersection = false;
        }
    }
  // Stop area bottom
    else if (t.transform.translation.x > 0.6 and t.transform.translation.x < 1.3 and t.transform.translation.y > -0.6 and t.transform.translation.y < 0.0) {
        RCLCPP_INFO(this->get_logger(), "Stop Area Bottom 1 detected");
        message.data = "FollowPath";
        if(new_intersection){
          vtl_request("0");
          new_intersection = false;
        }
    }
    // Stop area left
    else if (t.transform.translation.x > 1.3 and t.transform.translation.x < 2.0 and t.transform.translation.y > 0.6 and t.transform.translation.y < 1.4) {
        RCLCPP_INFO(this->get_logger(), "Stop Area Left 1 detected");
        message.data = "FollowPath";
        if(new_intersection){
          vtl_request("0");
          new_intersection = false;
        }
    }


  // Intersection 1
  // Intersection area 
    else if (t.transform.translation.x > -2.4 and t.transform.translation.x < -1.1 and t.transform.translation.y > -0.6 and t.transform.translation.y < 0.6) {
        RCLCPP_INFO(this->get_logger(), "Intersection Area 2 detected");
   
    }
  // Stop area right 
    else if (t.transform.translation.x > -1.75 and t.transform.translation.x < -1.1 and t.transform.translation.y > -1.35 and t.transform.translation.y < -0.7) {
        RCLCPP_INFO(this->get_logger(), "Stop Area Right 2 detected");
        message.data = "FollowPath";
        if(new_intersection){
          vtl_request("1");
          new_intersection = false;
        }
    }

  // Stop area top 
    else if (t.transform.translation.x > -1.1 and t.transform.translation.x < -0.4 and t.transform.translation.y > 0.0 and t.transform.translation.y < 0.6) {
        RCLCPP_INFO(this->get_logger(), "Stop Area Top 2 detected");
        message.data = "FollowPath";
        if(new_intersection){
          vtl_request("1");
          new_intersection = false;
        }
    }

  // Stop area left
    else if (t.transform.translation.x > -2.4 and t.transform.translation.x < -1.75 and t.transform.translation.y > 0.7 and t.transform.translation.y < 1.4) {
        RCLCPP_INFO(this->get_logger(), "Stop Area Left 2 detected");
        message.data = "FollowPath";
        if(new_intersection){
          vtl_request("1");
          new_intersection = false;
        }
    }
  
    else{
      // set the controller to lane controller
      message.data = "LaneController";
      
      if(switch_controller){
        vtl_free();
        switch_controller = false;
      }
    }
  } catch (const tf2::TransformException & ex) {
    // Intersection area 1
    RCLCPP_INFO(this->get_logger(), "No Transformation provided! Position checking not possible.");
  }   
  RCLCPP_DEBUG(this->get_logger(), "Publishing Controller: '%s'", message.data.c_str());
  publisher_->publish(message);
  return;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Intersection_Checker>());
  rclcpp::shutdown();
  return 0;
}