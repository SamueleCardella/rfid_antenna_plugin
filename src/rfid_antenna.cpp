#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rfid_msgs/msg/tag.hpp>
#include <rfid_msgs/msg/tag_array.hpp>
#include <cmath>
#include <cstdlib> 

#define MAX_RANGE 5
#define DEBUG 0
const double TWO_PI = 2 * M_PI;

namespace gazebo
{
  class MySensorPlugin : public ModelPlugin
  {
  public:
    MySensorPlugin() : ModelPlugin(), sensor_value_(0.0), update_rate_(10.0) {} // Default update rate of 2 Hz

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        // Ensure ROS2 node is initialized
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }

        // Create a ROS2 node
        node_ = std::make_shared<rclcpp::Node>("gazebo_client");


        // Store the model pointer
        model_ = _model;
        std::string rfidPubTopic = "ciccio";
        phi_0_ = 0.1;
        lambda_ = 0.1;
        radial_range_ = 0.1;
        P_T_ = 30;
        G_0_ = 5.5;
        //load parameters from sdf
        if(_sdf->HasElement("topic")) {
            rfidPubTopic = _sdf->Get<std::string>("topic");
        }
        if(_sdf->HasElement("radial_range")) {
            radial_range_ = _sdf->Get<double>("radial_range");
            std::string infoMsg = "Radial range = " + std::to_string(radial_range_);
            RCLCPP_INFO(node_->get_logger(), infoMsg.c_str());
        }
        if(_sdf->HasElement("lambda")) {
            lambda_ = _sdf->Get<double>("lambda");
        }
        if(_sdf->HasElement("phi_0")) {
            phi_0_ = _sdf->Get<double>("phi_0");
        }
        if(_sdf->HasElement("P_T")) {
            P_T_ = _sdf->Get<double>("P_T");
        }
        if(_sdf->HasElement("G_0")) {
            G_0_ = _sdf->Get<double>("G_0");
        }
        if(_sdf->HasElement("noise")) {
            auto _noiseElement =  _sdf->GetElement("noise");
            RCLCPP_INFO(node_->get_logger(), "Noise element found.");
            if(_noiseElement->HasElement("type")) {
              std::string noiseType;
              noiseType = _noiseElement->Get<std::string>("type");
                if(noiseType == "gaussian") {
                    RCLCPP_INFO(node_->get_logger(), "Gaussian noise type found.");
                    mean_ = _noiseElement->Get<float>("mean");
                    stddev_ = _noiseElement->Get<float>("stddev");
                    std::string infoMsg = "Mean = " + std::to_string(mean_) + " , StdDev = " + std::to_string(stddev_);
                    RCLCPP_INFO(node_->get_logger(), infoMsg.c_str());
                }    
            }
        }
        // Create a ROS2 publisher
        rfid_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("rfid/tag_position", 10);
        rfid_tag_pub_ = node_->create_publisher<rfid_msgs::msg::TagArray>("rfid/tag_array", 10);

        // Set the update period based on the desired update rate
        update_period_ = 1.0 / update_rate_;
        // Initialize the last update time
        last_update_time_ = model_->GetWorld()->SimTime();

        // Listen to the update event
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MySensorPlugin::OnUpdate, this));

        // Output a message
        RCLCPP_INFO(node_->get_logger(), "RfidAntenna loaded and ready.");
    }

    void OnUpdate()
    {
        // Get the current simulation time
        common::Time current_time = model_->GetWorld()->SimTime();

        // Check if it's time to update the sensor
        if ((current_time - last_update_time_).Double() >= update_period_) {
            auto mgsTimestamp = node_->now();
            visualization_msgs::msg::MarkerArray outputSensorMsg;
            rfid_msgs::msg::TagArray output;
            // Get the pose of the antenna (this plugin)
            // ignition::math::Pose3d antenna_pose = model_->GetLink("rfid_antenna_link")->WorldPose();
            ignition::math::Pose3d antenna_pose = model_->WorldPose();
            auto antenna_ptr = model_->GetLink("rfid_antenna_link");
            if(antenna_ptr == nullptr) {
                // std::cout << "SBIRRO NULLPTR" << std::endl;
            }
            // Get the Gazebo world
            gazebo::physics::WorldPtr world = gazebo::physics::get_world();

            // Iterate through objects of interest
            int32_t tagCounter = 0;
            for (physics::ModelPtr model : world->Models()) {
                std::string model_name = model->GetName();
                if (model_name.find("rfid_tag") != std::string::npos) {
                    ignition::math::Pose3d object_pose = model->WorldPose();
                    // Calculate distance between antenna and object
                    double distance = antenna_pose.Pos().Distance(object_pose.Pos());
                    ignition::math::Pose3d sensor_pose = model_->WorldPose();
                    // std::cout << "SENSOR POSE : \n" 
                    //           << "X = " << sensor_pose.X() << "\n"
                    //           << "Y = " << sensor_pose.Y() << "\n"
                    //           << "Z = " << sensor_pose.Z() << "\n"
                    //           << "YAW = " << sensor_pose.Yaw() << std::endl;

                    // Check if the object is within desired range or category
                    if (isTagInAntennaRange(antenna_pose, 
                                            object_pose, 
                                            distance)) {
                        // ignition::math::Pose3d relative_pose = object_pose - sensor_pose;
#if DEBUG

                        visualization_msgs::msg::Marker tagMarker;
                        tagMarker.pose.position.x = GenerateGaussianNoise(relative_pose.X());
                        tagMarker.pose.position.y = GenerateGaussianNoise(relative_pose.Y());
                        tagMarker.pose.position.z = GenerateGaussianNoise(relative_pose.Z());
                        tagMarker.pose.orientation.x = 0.0;
                        tagMarker.pose.orientation.y = 0.0;
                        tagMarker.pose.orientation.z = 0.0;
                        tagMarker.pose.orientation.w = 1.0;
                        tagMarker.scale.x = 0.1;
                        tagMarker.scale.y = 0.1;
                        tagMarker.scale.z = 0.1;
                        tagMarker.color.r = 1.0;
                        tagMarker.color.g = 0.0;
                        tagMarker.color.b = 0.0;
                        tagMarker.color.a = 1.0;
                        tagMarker.lifetime.sec = 0;
                        tagMarker.lifetime.nanosec = 0;
                        tagMarker.header.frame_id = "base_link";
                        tagMarker.header.stamp = mgsTimestamp;
                        tagMarker.type = 9;
                        tagMarker.action = 0;
                        tagMarker.id = tagCounter;
                        tagMarker.text = getTagIdFromName(model_name);
                        tagCounter ++;

                        outputSensorMsg.markers.push_back(tagMarker);
#endif
                        //Creation of the output message
                        output.n_tags ++;
                        rfid_msgs::msg::Tag singleTag;
                        singleTag.id = getTagIdFromName(model_name);
                        singleTag.distance = distance;
                        singleTag.phi = mod2PI(4*M_PI*distance*phi_0_/lambda_ + GenerateGaussianNoise(phi_0_));
                        //RSSI computation
                        ignition::math::Vector3d rel_vec = object_pose.Pos() - antenna_pose.Pos();
                        double theta_R = acos(rel_vec.Z() / rel_vec.Length());
                        double phi_R = atan2(rel_vec.Y(), rel_vec.X());
                        double G_t = G_0_ * pow(cos(theta_R), 10);
                        double rssi = P_T_ + G_t - 20 * log10(4 * M_PI * distance / lambda_) + GenerateGaussianNoise(0);
                        singleTag.rssi = rssi;

                        output.tags.push_back(singleTag);
                    }
                }
            }

            if(outputSensorMsg.markers.size() > 0) {
                rfid_pub_->publish(outputSensorMsg);
            }
            if(output.tags.size() > 0) {
                output.header.stamp = mgsTimestamp;
                output.header.frame_id = "antenna";
                rfid_tag_pub_->publish(output);
            }

            // Optionally, output to ROS console for debugging
            // RCLCPP_INFO(node_->get_logger(), "Sensor value: %f", sensor_value_);

            // Update the last update time
            last_update_time_ = current_time;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rfid_pub_;
    rclcpp::Publisher<rfid_msgs::msg::TagArray>::SharedPtr rfid_tag_pub_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    double sensor_value_;
    double update_rate_;
    double update_period_;
    double radial_range_;
    double lambda_;
    double phi_0_;
    common::Time last_update_time_;
    std::normal_distribution<> noise_model_;
    float mean_;
    float stddev_;
    double P_T_;
    double G_0_;

	float GenerateGaussianNoise(const float& value) {
		float u1 = static_cast<float>(rand()) / RAND_MAX;
		float u2 = static_cast<float>(rand()) / RAND_MAX;

		float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
		return value + z0 * stddev_ + mean_;
	}

	std::string getTagIdFromName(const std::string& tagName) {
		size_t pos = tagName.find_last_of("_");
		std::string retVal = "" ;
		if(pos != tagName.npos) {
			retVal = tagName.substr(pos + 1);
		}
		return retVal;
	}

    double mod2PI(const double& angle) {
        // Use fmod to get the remainder of angle / 2π
        double result = fmod(angle, TWO_PI);
        // fmod can return negative values, so we ensure the result is positive
        if (result < 0) {
            result += TWO_PI;
        }
        return result;
    }

    double modPI(const double& angle) {
        // Use fmod to get the remainder of angle / 2π
        double result = fmod(angle, M_PI);
        // fmod can return negative values, so we ensure the result is positive
        if (result < 0) {
            result += M_PI;
        }
        return result;
    }

    bool isTagInAntennaRange (const ignition::math::Pose3d& antennaPoseInWorld, 
                              const ignition::math::Pose3d& tagPoseInWorld, 
                              const double& distance){
        bool retVal = false;
        if(distance > MAX_RANGE) {
            return retVal;
        }
        // Step 4: Get the sensor's orientation (as a quaternion) and extract the forward direction
        ignition::math::Quaterniond antennaRot = antennaPoseInWorld.Rot();
        ignition::math::Vector3d antennaMeasurementAxis(1.0, 0.0, 0.0);  // Assume the sensor faces along its local Z-axis
        ignition::math::Vector3d antennaDirInWorld = antennaRot.RotateVector(antennaMeasurementAxis);  // Rotate into world frame

        //compute sensor to object vector
        // ignition::math::Vector3d antennaToTag = antennaPoseInWorld.Pos() - tagPoseInWorld.Pos();
        ignition::math::Vector3d antennaToTag = tagPoseInWorld.Pos() - antennaPoseInWorld.Pos();
    
        //normalize sensor to object and antenna dir in world vectors
        ignition::math::Vector3d antennaToTagNorm = antennaToTag/distance;
        ignition::math::Vector3d antennaDirInWorldNorm = antennaDirInWorld/sqrt(antennaDirInWorld.X()*antennaDirInWorld.X() + 
                                                                                    antennaDirInWorld.Y()*antennaDirInWorld.Y() +
                                                                                    antennaDirInWorld.Z()*antennaDirInWorld.Z());
    
        //compute the dot product between the two vectors
        double dotProduct = antennaToTagNorm.X()*antennaDirInWorldNorm.X() + 
                            antennaToTagNorm.Y()*antennaDirInWorldNorm.Y() +
                            antennaToTagNorm.Z()*antennaDirInWorldNorm.Z();

        //compute angle between the two vectors
        double theta = modPI(acos(dotProduct));// - M_PI_2;
        // std::cout << "Theta = " << theta << std::endl;
        if(theta <= radial_range_) {
            retVal = true;
        }
        return retVal;
    }    
  };

  GZ_REGISTER_MODEL_PLUGIN(MySensorPlugin)
}

