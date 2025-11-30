#include <openxr/openxr.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <cstring>
#include <stdexcept>
#include <memory>

// Structure to hold pose data
struct PoseData {
    float position[3];
    float orientation[4];
    XrTime time;
};

class OpenXRPosePublisher : public rclcpp::Node {
private:
    // OpenXR members
    XrInstance instance;
    XrSystemId systemId;
    XrSession session;
    XrSpace viewSpace;
    XrSpace stageSpace;
    bool initialized;
    bool sessionCreated;

    // ROS2 members
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    int frame_count_;

    void checkXR(XrResult res, const char* call) {
        if (XR_FAILED(res)) {
            RCLCPP_ERROR(this->get_logger(), "XR Error: %s = %d", call, res);
            throw std::runtime_error("OpenXR call failed");
        }
    }

public:
    OpenXRPosePublisher() : Node("openxr_pose_publisher"),
                             instance(XR_NULL_HANDLE), 
                             systemId(XR_NULL_SYSTEM_ID), 
                             session(XR_NULL_HANDLE),
                             viewSpace(XR_NULL_HANDLE),
                             stageSpace(XR_NULL_HANDLE),
                             initialized(false),
                             sessionCreated(false),
                             frame_count_(0) {
        
        // Declare parameters
        this->declare_parameter("publish_rate", 60.0);
        this->declare_parameter("topic_name", "vr/headset_pose");
        this->declare_parameter("frame_id", "vr_stage");
        this->declare_parameter("child_frame_id", "vr_headset");
        this->declare_parameter("publish_tf", true);
        
        // Get parameters
        double publish_rate = this->get_parameter("publish_rate").as_double();
        std::string topic_name = this->get_parameter("topic_name").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Starting OpenXR Pose Publisher...");
        
        // Initialize OpenXR
        initializeOpenXR();
        createSession();
        
        // Create ROS2 publisher
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // Create timer for periodic publishing
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&OpenXRPosePublisher::publishPose, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Publishing headset pose to: %s at %.1f Hz", 
                    topic_name.c_str(), publish_rate);
    }
    
    ~OpenXRPosePublisher() {
        cleanup();
    }

private:
    void initializeOpenXR() {
        RCLCPP_INFO(this->get_logger(), "Initializing OpenXR...");
        
        // Create OpenXR instance
        XrInstanceCreateInfo createInfo{};
        createInfo.type = XR_TYPE_INSTANCE_CREATE_INFO;
        createInfo.next = nullptr;
        createInfo.createFlags = 0;
        strcpy(createInfo.applicationInfo.applicationName, "ROS2 OpenXR Publisher");
        createInfo.applicationInfo.applicationVersion = 1;
        strcpy(createInfo.applicationInfo.engineName, "ROS2");
        createInfo.applicationInfo.engineVersion = 1;
        createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
        createInfo.enabledApiLayerCount = 0;
        createInfo.enabledApiLayerNames = nullptr;
        createInfo.enabledExtensionCount = 0;
        createInfo.enabledExtensionNames = nullptr;
        
        checkXR(xrCreateInstance(&createInfo, &instance), "xrCreateInstance");
        RCLCPP_INFO(this->get_logger(), "✓ OpenXR instance created");
        
        // Get VR system
        XrSystemGetInfo systemInfo{};
        systemInfo.type = XR_TYPE_SYSTEM_GET_INFO;
        systemInfo.next = nullptr;
        systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
        
        checkXR(xrGetSystem(instance, &systemInfo, &systemId), "xrGetSystem");
        RCLCPP_INFO(this->get_logger(), "✓ VR system detected");
        
        // Print system properties
        XrSystemProperties systemProps{};
        systemProps.type = XR_TYPE_SYSTEM_PROPERTIES;
        systemProps.next = nullptr;
        xrGetSystemProperties(instance, systemId, &systemProps);
        RCLCPP_INFO(this->get_logger(), "✓ System: %s", systemProps.systemName);
        
        initialized = true;
    }

    void createSession() {
        if (!initialized) {
            throw std::runtime_error("Must initialize OpenXR first");
        }

        RCLCPP_INFO(this->get_logger(), "Creating OpenXR session...");
        
        // Create session
        XrSessionCreateInfo sessionInfo{};
        sessionInfo.type = XR_TYPE_SESSION_CREATE_INFO;
        sessionInfo.next = nullptr;
        sessionInfo.createFlags = 0;
        sessionInfo.systemId = systemId;
        
        checkXR(xrCreateSession(instance, &sessionInfo, &session), "xrCreateSession");
        RCLCPP_INFO(this->get_logger(), "✓ Session created");

        // Create reference spaces
        XrReferenceSpaceCreateInfo spaceInfo{};
        spaceInfo.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO;
        spaceInfo.next = nullptr;
        spaceInfo.poseInReferenceSpace.orientation.x = 0.0f;
        spaceInfo.poseInReferenceSpace.orientation.y = 0.0f;
        spaceInfo.poseInReferenceSpace.orientation.z = 0.0f;
        spaceInfo.poseInReferenceSpace.orientation.w = 1.0f;
        spaceInfo.poseInReferenceSpace.position.x = 0.0f;
        spaceInfo.poseInReferenceSpace.position.y = 0.0f;
        spaceInfo.poseInReferenceSpace.position.z = 0.0f;
        
        // View space (tracks the headset)
        spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
        checkXR(xrCreateReferenceSpace(session, &spaceInfo, &viewSpace), "xrCreateReferenceSpace(VIEW)");
        
        // Stage space (room-scale tracking)
        spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
        checkXR(xrCreateReferenceSpace(session, &spaceInfo, &stageSpace), "xrCreateReferenceSpace(STAGE)");
        
        RCLCPP_INFO(this->get_logger(), "✓ Reference spaces created");

        // Begin session
        XrSessionBeginInfo beginInfo{};
        beginInfo.type = XR_TYPE_SESSION_BEGIN_INFO;
        beginInfo.next = nullptr;
        beginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        checkXR(xrBeginSession(session, &beginInfo), "xrBeginSession");
        RCLCPP_INFO(this->get_logger(), "✓ Session started");

        sessionCreated = true;
    }

    PoseData getHeadsetPose() {
        if (!sessionCreated) {
            throw std::runtime_error("Session not created");
        }

        PoseData pose;
        
        // Locate the view space in stage space
        XrSpaceLocation spaceLocation{};
        spaceLocation.type = XR_TYPE_SPACE_LOCATION;
        spaceLocation.next = nullptr;
        XrTime displayTime = 0; // Get latest pose
        
        checkXR(xrLocateSpace(viewSpace, stageSpace, displayTime, &spaceLocation), "xrLocateSpace");
        
        // Check tracking validity
        bool positionValid = (spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;
        bool orientationValid = (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
        
        if (!positionValid) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Position tracking lost");
        }
        if (!orientationValid) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Orientation tracking lost");
        }
        
        // Extract position
        pose.position[0] = spaceLocation.pose.position.x;
        pose.position[1] = spaceLocation.pose.position.y;
        pose.position[2] = spaceLocation.pose.position.z;
        
        // Extract orientation (quaternion)
        pose.orientation[0] = spaceLocation.pose.orientation.x;
        pose.orientation[1] = spaceLocation.pose.orientation.y;
        pose.orientation[2] = spaceLocation.pose.orientation.z;
        pose.orientation[3] = spaceLocation.pose.orientation.w;
        
        pose.time = displayTime;
        
        return pose;
    }

    void publishPose() {
        try {
            // Get current headset pose
            PoseData pose = getHeadsetPose();
            
            // Create timestamp
            auto timestamp = this->now();
            
            // Get parameters
            std::string frame_id = this->get_parameter("frame_id").as_string();
            std::string child_frame_id = this->get_parameter("child_frame_id").as_string();
            bool publish_tf = this->get_parameter("publish_tf").as_bool();
            
            // Publish PoseStamped message
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = timestamp;
            pose_msg.header.frame_id = frame_id;
            
            // Position
            pose_msg.pose.position.x = pose.position[0];
            pose_msg.pose.position.y = pose.position[1];
            pose_msg.pose.position.z = pose.position[2];
            
            // Orientation (quaternion)
            pose_msg.pose.orientation.x = pose.orientation[0];
            pose_msg.pose.orientation.y = pose.orientation[1];
            pose_msg.pose.orientation.z = pose.orientation[2];
            pose_msg.pose.orientation.w = pose.orientation[3];
            
            pose_publisher_->publish(pose_msg);
            
            // Publish TF transform
            if (publish_tf) {
                geometry_msgs::msg::TransformStamped transform;
                transform.header.stamp = timestamp;
                transform.header.frame_id = frame_id;
                transform.child_frame_id = child_frame_id;
                
                transform.transform.translation.x = pose.position[0];
                transform.transform.translation.y = pose.position[1];
                transform.transform.translation.z = pose.position[2];
                
                transform.transform.rotation.x = pose.orientation[0];
                transform.transform.rotation.y = pose.orientation[1];
                transform.transform.rotation.z = pose.orientation[2];
                transform.transform.rotation.w = pose.orientation[3];
                
                tf_broadcaster_->sendTransform(transform);
            }
            
            frame_count_++;
            if (frame_count_ % 60 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "Published %d poses | Pos: [%.3f, %.3f, %.3f]", 
                    frame_count_, 
                    pose.position[0], pose.position[1], pose.position[2]);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error publishing pose: %s", e.what());
        }
    }

    void cleanup() {
        if (viewSpace != XR_NULL_HANDLE) {
            xrDestroySpace(viewSpace);
            viewSpace = XR_NULL_HANDLE;
        }
        if (stageSpace != XR_NULL_HANDLE) {
            xrDestroySpace(stageSpace);
            stageSpace = XR_NULL_HANDLE;
        }
        if (session != XR_NULL_HANDLE) {
            xrEndSession(session);
            xrDestroySession(session);
            session = XR_NULL_HANDLE;
            RCLCPP_INFO(this->get_logger(), "✓ Session closed");
        }
        if (instance != XR_NULL_HANDLE) {
            xrDestroyInstance(instance);
            instance = XR_NULL_HANDLE;
            RCLCPP_INFO(this->get_logger(), "✓ Cleanup complete");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<OpenXRPosePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}


// ========================================
// Run the node:
// ========================================
// ros2 run <package_name> openxr_pose_publisher
//
// With custom parameters:
// ros2 run <package_name> openxr_pose_publisher --ros-args \
//   -p publish_rate:=90.0 \
//   -p topic_name:=vr/headset_pose \
//   -p frame_id:=world \
//   -p child_frame_id:=headset \
//   -p publish_tf:=true

// ========================================
// View the published data:
// ========================================
// # Echo pose messages
// ros2 topic echo /vr/headset_pose
//
// # Check publish rate
// ros2 topic hz /vr/headset_pose
//
// # View TF tree
// ros2 run tf2_tools view_frames
//
// # Echo TF transform
// ros2 run tf2_ros tf2_echo vr_stage vr_headset
//
// # Visualize in RViz
// rviz2
// # Add: TF display, set Fixed Frame to "vr_stage"