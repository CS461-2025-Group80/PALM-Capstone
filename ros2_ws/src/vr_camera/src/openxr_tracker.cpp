
// openxr_tracker.cpp
#include "openxr_tracker.h"
#include <cstring>
#include <iostream>

OpenXRTracker::OpenXRTracker() {}

OpenXRTracker::~OpenXRTracker() {
    cleanup();
}

bool OpenXRTracker::initialize() {
    if (!createInstance()) return false;
    if (!createSession()) return false;
    if (!createReferenceSpaces()) return false;
    if (!beginSession()) return false;
    return true;
}

bool OpenXRTracker::createInstance() {
    XrInstanceCreateInfo createInfo{XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy(createInfo.applicationInfo.applicationName, "Quest-Tracker");
    createInfo.applicationInfo.applicationVersion = 1;
    strcpy(createInfo.applicationInfo.engineName, "None");
    createInfo.applicationInfo.engineVersion = 1;
    createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    
    XrResult result = xrCreateInstance(&createInfo, &instance_);
    if (result != XR_SUCCESS) {
        lastError_ = "Failed to create XR instance: " + std::to_string(result);
        return false;
    }
    
    // Get system
    XrSystemGetInfo systemInfo{XR_TYPE_SYSTEM_GET_INFO};
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    
    result = xrGetSystem(instance_, &systemInfo, &systemId_);
    if (result != XR_SUCCESS) {
        lastError_ = "Failed to get XR system: " + std::to_string(result);
        return false;
    }
    
    return true;
}

bool OpenXRTracker::createSession() {
    XrSessionCreateInfo sessionInfo{XR_TYPE_SESSION_CREATE_INFO};
    sessionInfo.systemId = systemId_;
    
    XrResult result = xrCreateSession(instance_, &sessionInfo, &session_);
    if (result != XR_SUCCESS) {
        lastError_ = "Failed to create XR session: " + std::to_string(result);
        return false;
    }
    
    return true;
}

bool OpenXRTracker::createReferenceSpaces() {
    // Create VIEW space (head tracking)
    XrReferenceSpaceCreateInfo spaceInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    spaceInfo.poseInReferenceSpace.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
    spaceInfo.poseInReferenceSpace.position = {0.0f, 0.0f, 0.0f};
    
    XrResult result = xrCreateReferenceSpace(session_, &spaceInfo, &viewSpace_);
    if (result != XR_SUCCESS) {
        lastError_ = "Failed to create view space: " + std::to_string(result);
        return false;
    }
    
    // Create LOCAL space for reference
    spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    result = xrCreateReferenceSpace(session_, &spaceInfo, &localSpace_);
    if (result != XR_SUCCESS) {
        lastError_ = "Failed to create local space: " + std::to_string(result);
        return false;
    }
    
    return true;
}

bool OpenXRTracker::beginSession() {
    XrSessionBeginInfo beginInfo{XR_TYPE_SESSION_BEGIN_INFO};
    beginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    
    XrResult result = xrBeginSession(session_, &beginInfo);
    if (result != XR_SUCCESS) {
        lastError_ = "Failed to begin XR session: " + std::to_string(result);
        return false;
    }
    
    sessionRunning_ = true;
    return true;
}

void OpenXRTracker::pollEvents() {
    XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(instance_, &event) == XR_SUCCESS) {
        switch (event.type) {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                auto stateEvent = reinterpret_cast<XrEventDataSessionStateChanged*>(&event);
                sessionState_ = stateEvent->state;
                break;
            }
            default:
                break;
        }
        event = {XR_TYPE_EVENT_DATA_BUFFER};
    }
}

bool OpenXRTracker::update(OrientationData& data) {
    if (!sessionRunning_) return false;
    
    // Poll for events
    pollEvents();
    
    // Wait for next frame
    if (xrWaitFrame(session_, nullptr, &frameState_) != XR_SUCCESS) {
        return false;
    }
    
    // Begin frame
    if (xrBeginFrame(session_, nullptr) != XR_SUCCESS) {
        return false;
    }
    
    // Get current pose
    XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};
    XrResult result = xrLocateSpace(viewSpace_, localSpace_, 
                                    frameState_.predictedDisplayTime, &location);
    
    if (result == XR_SUCCESS) {
        data.orientation_valid = 
            (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
        data.position_valid = 
            (location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;
        
        if (data.orientation_valid) {
            data.x = location.pose.orientation.x;
            data.y = location.pose.orientation.y;
            data.z = location.pose.orientation.z;
            data.w = location.pose.orientation.w;
        }
        
        if (data.position_valid) {
            data.pos_x = location.pose.position.x;
            data.pos_y = location.pose.position.y;
            data.pos_z = location.pose.position.z;
        }
        
        data.timestamp = frameState_.predictedDisplayTime;
    }
    
    // End frame
    XrFrameEndInfo endInfo{XR_TYPE_FRAME_END_INFO};
    endInfo.displayTime = frameState_.predictedDisplayTime;
    endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    xrEndFrame(session_, &endInfo);
    
    return result == XR_SUCCESS;
}

void OpenXRTracker::cleanup() {
    sessionRunning_ = false;
    
    if (viewSpace_ != XR_NULL_HANDLE) {
        xrDestroySpace(viewSpace_);
        viewSpace_ = XR_NULL_HANDLE;
    }
    
    if (localSpace_ != XR_NULL_HANDLE) {
        xrDestroySpace(localSpace_);
        localSpace_ = XR_NULL_HANDLE;
    }
    
    if (session_ != XR_NULL_HANDLE) {
        xrDestroySession(session_);
        session_ = XR_NULL_HANDLE;
    }
    
    if (instance_ != XR_NULL_HANDLE) {
        xrDestroyInstance(instance_);
        instance_ = XR_NULL_HANDLE;
    }
}
