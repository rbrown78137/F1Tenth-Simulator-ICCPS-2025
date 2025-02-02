#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/msgs/msgs.hh>
#include <random>
#include <chrono>
#include <ros/ros.h>
#include <ros/param.h>

std::string global_variable = "";
gazebo::physics::WorldPtr global_world_ptr;
gazebo::physics::ModelPtr f1tenth_model;
gazebo::physics::ModelPtr red_car_model;

std::default_random_engine generator;
std::normal_distribution<double> distribution(0,1);

float f1tenth_car_velocity = 0;
float f1tenth_car_steering_angle = 0;

float red_car_velocity = 0;
float red_car_steering_angle = 0;

float f1tenth_car_x = 0;
float f1tenth_car_y = 0;
float f1tenth_car_z = 0;

float red_car_x = 0;
float red_car_y = 0;
float red_car_z = 0;

// float MAX_SAMPLE_TIME = 6.0; // seconds
float MAX_SAMPLE_TIME = 4.0; // seconds
int number_of_collisions = 0;
int samples_counted = 0;

int MAX_SAMPLES = 1000;
bool SIMULATION_OVER = false;
float last_time_start = 0;
auto start = std::chrono::high_resolution_clock::now();
auto end = std::chrono::high_resolution_clock::now();
int collision_sample = -1;
    
namespace gazebo
{
void generate_new_poses();
void contact_callback(ConstContactsPtr &message);
void pose_callback(ConstPosesStampedPtr &message);
void SetVelocity(double velocity,gazebo::physics::ModelPtr model,std::string modelName);
void SetSteeringAngle(double angle,gazebo::physics::ModelPtr model,std::string modelName);

std::string f1tenth_collision_array[] = {
  "f1tenth_car::base::collision",
  "f1tenth_car::front_left_wheel::collision",
  "f1tenth_car::front_right_wheel::collision",
  "f1tenth_car::back_left_wheel::collision",
  "f1tenth_car::back_right_wheel::collision"
};
std::string red_car_collision_array[] = {
  "red_car::base::collision",
  "red_car::front_left_wheel::collission",
  "red_car::front_right_wheel::collision",
  "red_car::back_left_wheel::collision",
  "red_car::back_right_wheel::collision"
};
void generate_new_poses(){
  float f1tenth_car_x_mean = 0; float f1tenth_car_x_std = 0;
  float f1tenth_car_y_mean = 0; float f1tenth_car_y_std = 0;
  float f1tenth_car_velocity_mean = 0; float f1tenth_car_velocity_std = 0;
  float f1tenth_car_steering_angle_mean = 0; float f1tenth_car_steering_angle_std = 0;

  float red_car_x_mean = 0; float red_car_x_std = 0;
  float red_car_y_mean = 0.0; float red_car_y_std = 0;
  float red_car_velocity_mean = 0; float red_car_velocity_std = 0;
  float red_car_steering_angle_mean = 0; float red_car_steering_angle_std = 0;
  
  if(collision_sample == 1){
    // X AND Y ARE FLOPPED IN THIS EXAMPLE
    f1tenth_car_velocity_mean = 1;
    red_car_x_mean = 4 + 0.540363;
    red_car_x_std = 0.3;
  }
  if(collision_sample == 2){
    red_car_x_mean = 2 + 0.540363;
    f1tenth_car_velocity_mean = 1;
    red_car_y_mean = 3 * 0.298078;
    red_car_y_std = 0.298078;

  }
  // float f1tenth_car_x_mean = 0; float f1tenth_car_x_std = 0;
  // float f1tenth_car_y_mean = 0; float f1tenth_car_y_std = 0;
  // float f1tenth_car_velocity_mean = 1; float f1tenth_car_velocity_std = 0;
  // float f1tenth_car_steering_angle_mean = 0; float f1tenth_car_steering_angle_std = 0;

  // float red_car_x_mean = 2+ 0.540363; float red_car_x_std = 0;
  // float red_car_y_mean = 0.298078+0.298078; float red_car_y_std = 0.298078;
  // float red_car_velocity_mean = 0; float red_car_velocity_std = 0;
  // float red_car_steering_angle_mean = 0; float red_car_steering_angle_std = 0;

  f1tenth_car_x = f1tenth_car_x_mean + distribution(generator) * f1tenth_car_x_std;
  f1tenth_car_y = f1tenth_car_y_mean + distribution(generator) * f1tenth_car_y_std;
  f1tenth_car_velocity = f1tenth_car_velocity_mean + distribution(generator) * f1tenth_car_velocity_std;
  f1tenth_car_steering_angle = f1tenth_car_steering_angle_mean + distribution(generator) * f1tenth_car_steering_angle_std;

  red_car_x = red_car_x_mean + distribution(generator) * red_car_x_std;
  red_car_y = red_car_y_mean + distribution(generator) * red_car_y_std;
  red_car_velocity = red_car_velocity_mean + distribution(generator) * red_car_velocity_std;
  red_car_steering_angle = red_car_steering_angle_mean + distribution(generator) * red_car_steering_angle_std;
  // double number = distribution(generator);
  // std::cout<<number<<std::endl;
  ignition::math::Vector3d zero_vector;
  zero_vector.Set(0.0,0.0,0.0);
  red_car_model->SetWorldTwist(zero_vector,zero_vector);
  f1tenth_model->SetWorldTwist(zero_vector,zero_vector);

  ignition::math::Pose3d f1tenth_car_pose;
  f1tenth_car_pose.SetX(f1tenth_car_x);
  f1tenth_car_pose.SetY(f1tenth_car_y);
  f1tenth_car_pose.SetZ(0.0);
  f1tenth_model->SetWorldPose(f1tenth_car_pose);

  ignition::math::Pose3d red_car_pose;
  red_car_pose.SetX(red_car_x);
  red_car_pose.SetY(red_car_y);
  red_car_pose.SetZ(0.0);
  red_car_model->SetWorldPose(red_car_pose);

  f1tenth_model->ResetPhysicsStates();
  red_car_model->ResetPhysicsStates();
  SetVelocity(f1tenth_car_velocity,f1tenth_model,"f1tenth_car");
  SetSteeringAngle(f1tenth_car_steering_angle ,f1tenth_model,"f1tenth_car");
  SetVelocity(red_car_velocity,red_car_model,"red_car");
  SetSteeringAngle(red_car_steering_angle ,red_car_model,"red_car");
}
void contact_callback(ConstContactsPtr &message){
    int contact_size = message->contact_size();
    bool is_contact = false;
    for (int i=0;i<contact_size;i++){
        gazebo::msgs::Contact contact = message->contact(i);
        std::string first_contact = contact.collision1();
        std::string second_contact = contact.collision2();
        for (int i = 0; i<5;i++){
          for (int j = 0; j<5;j++){
            if (
              (first_contact == f1tenth_collision_array[i] && second_contact == red_car_collision_array[j]) ||
              (second_contact == f1tenth_collision_array[i] && first_contact == red_car_collision_array[j])
              ){
              is_contact = true;
            }
          }
        }
    }
    if (is_contact){
      float current_time = message->time().sec() + (message->time().nsec() / 1.0e9);
      if(abs(current_time-last_time_start) > 0.02){
        last_time_start = current_time;
        number_of_collisions++;
        samples_counted++;
        generate_new_poses();
      }
    }
    // auto test2 = message->contact(0);   // std::cout<<"----\n"<<std::endl;
    //std::cout<<"----\n"<<std::endl;
    // std::cout << message->DebugString()<<std::endl;
}

void pose_callback(ConstPosesStampedPtr &message){
  // std::cout<<     // std::cout<<"HIT CALLBACK"<<std::endl;
    // std::cout << global_variable;
    // std::cout<<"----\n"<<std::endl;
    // std::cout<<message;
    // std::string test = message->DebugString();
    // std::cout<<test;e-){
  float current_time = message->time().sec() + (message->time().nsec() / 1.0e9);
  if(current_time-last_time_start > MAX_SAMPLE_TIME){
    last_time_start = current_time;
    samples_counted++;
    generate_new_poses();
  }
  if(samples_counted < MAX_SAMPLES){
    std::cout << "TRIAL " << samples_counted << " out of " << MAX_SAMPLES<< std::endl;
  }
  if(!SIMULATION_OVER && samples_counted >= MAX_SAMPLES){
    end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Duration: " << duration.count() << " milliseconds" << std::endl;
    std::cout<<"NUMBER OF SIMULATIONS: "<<samples_counted << std::endl;
    std::cout<<"NUMBER OF COLLISIONS: "<< number_of_collisions <<std::endl;
    SIMULATION_OVER = true;
  }
  // std::cout<< (message->time().sec() + (message->time().nsec() / 1.0e9 ));
  // std::cout<<"---"<<std::endl;
  gazebo::physics::ModelPtr f1tenth_model = global_world_ptr->ModelByName("f1tenth_car");
  SetVelocity(f1tenth_car_velocity,f1tenth_model,"f1tenth_car");
  SetSteeringAngle(f1tenth_car_steering_angle ,f1tenth_model,"f1tenth_car");
  SetVelocity(red_car_velocity,red_car_model,"red_car");
  SetSteeringAngle(red_car_steering_angle ,red_car_model,"red_car");
  // std::cout << message->time().Utf8DebugString();
}
void SetVelocity(double velocity,gazebo::physics::ModelPtr model,std::string modelName)
  {
    if(isnan(velocity)){
      velocity = 0;
    }
    if(isinf(velocity)){
      velocity = 0;
    }
    if(velocity<0){
      velocity = 0;
    }
    if(velocity> 7){
      velocity = 7;
    }
    model->GetJoint((modelName +"::back_left_joint"))->SetParam("fmax", 0, 2.8);
    model->GetJoint((modelName +"::back_left_joint"))->SetParam("vel", 0, 20 * velocity); // 20 converts m/s input to rad/s that wheels need
    model->GetJoint((modelName +"::back_right_joint"))->SetParam("fmax", 0, 2.8);
    model->GetJoint((modelName +"::back_right_joint"))->SetParam("vel", 0, 20 * velocity);
    model->GetJoint((modelName +"::front_left_joint"))->SetParam("fmax", 1, 2.8);
    model->GetJoint((modelName +"::front_left_joint"))->SetParam("vel", 0, 20 * velocity);
    model->GetJoint((modelName +"::front_right_joint"))->SetParam("fmax", 1, 2.8);
    model->GetJoint((modelName +"::front_right_joint"))->SetParam("vel", 0, 20 * velocity);
  }
void SetSteeringAngle(double angle,gazebo::physics::ModelPtr model,std::string modelName){
  if(isnan(angle)){
    angle = 0;
  }
  if(isinf(angle)){
    angle = 0;
  }
  double maximum_angle = 0.9;
  double turning_angle = angle;
  if(abs(angle)>=maximum_angle){//51 degrees
      turning_angle = angle / abs(angle) *maximum_angle;
  }
  (model)->GetJointController()->SetJointPosition((modelName +"::front_left_joint_turning"), turning_angle);
  (model)->GetJointController()->SetJointPosition((modelName +"::front_right_joint_turning"), turning_angle);
  }
class CollisionSample : public WorldPlugin
{
    private: gazebo::transport::NodePtr node;
    private: gazebo::transport::SubscriberPtr sub_contact;
    private: gazebo::transport::SubscriberPtr sub_pose;
    // public: static physics::WorldPtr world;
    public: void Load(physics::WorldPtr parent, sdf::ElementPtr /*_sdf*/)
    {
      if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "collision_sample_plugin", ros::init_options::NoSigintHandler);
      }
      ros::NodeHandle nh;
      nh.param("simulation_example", collision_sample, 1); 
      std::cout<<collision_sample<<std::endl;
      std::cout<<"Loading simulation...";
      std::cout<<std::endl;
      physics::WorldPtr world_ptr = parent;
      global_variable = parent->Name();
      global_world_ptr = world_ptr;
      // CollisionSample::world = parent;
      f1tenth_model = world_ptr->ModelByName("f1tenth_car");
      red_car_model = world_ptr->ModelByName("red_car");
      generate_new_poses();
      start = std::chrono::high_resolution_clock::now();
      // ignition::math::Pose3d f1tenth_pose;
      // f1tenth_pose.SetX(f1tenth_car_x);
      // f1tenth_pose.SetY(f1tenth_car_y);
      // f1tenth_pose.SetZ(f1tenth_car_z);
      // ignition::math::Pose3d red_car_pose;
      // red_car_pose.SetX(red_car_x);
      // red_car_pose.SetY(red_car_y);
      // red_car_pose.SetZ(red_car_z);
      // f1tenth_model->SetWorldPose(f1tenth_pose);
      // red_car_model->SetWorldPose(red_car_pose);
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();
      this->sub_contact = this->node->Subscribe("/car_contact",contact_callback);
      this->sub_pose = this->node->Subscribe("/gazebo/default/pose/info",pose_callback);
      if (this->sub_contact)
      {
      std::cout << "Failed to subscribe to [" << "/car_contact" << "]\n"<<std::endl;
      }
      if (this->sub_contact)
      {
      std::cout << " subscribed to [" << "/car_contact" << "]\n"<<std::endl;
      }
    }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CollisionSample)
}