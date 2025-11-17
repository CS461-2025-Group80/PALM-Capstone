// openxr_tracker.h
#pragma once
#include <openxr/openxr.h>
#include <functional>
#include <string>

struct OrientationData {
    double x, y, z, w;  // Quaternion
    double pos_x, pos_y, pos_z;  // Position
    uint64_t timestamp;
    bool orientation_valid;
    bool position_valid;
};

class OpenXRTracker {
public:
    OpenXRTracker();
    ~OpenXRTracker();
    
    // Initialize OpenXR session
    bool initialize();
    
    // Update and get latest orientation data
    bool update(OrientationData& data);
    
    // Check if tracker is running
    bool isRunning() const { return session_ != XR_NULL_HANDLE; }
    
    // Get error message if initialization fails
    std::string getLastError() const { return lastError_; }
    
    // Cleanup resources
    void cleanup();

private:
    bool createInstance();
    bool createSession();
    bool createReferenceSpaces();
    bool beginSession();
    void pollEvents();
    
    XrInstance instance_ = XR_NULL_HANDLE;
    XrSystemId systemId_ = XR_NULL_SYSTEM_ID;
    XrSession session_ = XR_NULL_HANDLE;
    XrSpace viewSpace_ = XR_NULL_HANDLE;
    XrSpace localSpace_ = XR_NULL_HANDLE;
    XrFrameState frameState_{XR_TYPE_FRAME_STATE};
    XrSessionState sessionState_ = XR_SESSION_STATE_UNKNOWN;
    bool sessionRunning_ = false;
    
    std::string lastError_;
};