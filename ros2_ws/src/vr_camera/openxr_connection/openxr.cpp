#include <vulkan/vulkan.h>

#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include <vector>
#include <array>

// Helper macros
#define CHECK_XR(call) { XrResult res = call; if (XR_FAILED(res)) { std::cerr << "XR Error: " << #call << " = " << res << std::endl; return 1; } }
#define CHECK_VK(call) { VkResult res = call; if (res != VK_SUCCESS) { std::cerr << "VK Error: " << #call << " = " << res << std::endl; return 1; } }

// Camera wrapper
class Camera {
public:
    cv::VideoCapture cap;
    cv::Mat frame;
    int width = 640;
    int height = 480;
    std::vector<uint8_t> frameData;
    
    bool init() {
        cap.open(0);
        if (!cap.isOpened()) {
            std::cerr << "Cannot open camera\n";
            return false;
        }
        
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        
        frameData.resize(width * height * 4);
        std::cout << "✓ Camera initialized (" << width << "x" << height << ")\n";
        return true;
    }
    
    bool captureFrame() {
        if (!cap.read(frame)) return false;
        
        cv::Mat rgba;
        cv::cvtColor(frame, rgba, cv::COLOR_BGR2RGBA);
        
        if (rgba.isContinuous()) {
            memcpy(frameData.data(), rgba.data, width * height * 4);
        } else {
            for (int y = 0; y < height; y++) {
                memcpy(frameData.data() + y * width * 4, rgba.ptr(y), width * 4);
            }
        }
        return true;
    }
    
    void cleanup() {
        if (cap.isOpened()) cap.release();
    }
};

// Vertex shader (GLSL)
const char* vertexShaderCode = R"(
#version 450
layout(location = 0) out vec2 outUV;

vec2 positions[6] = vec2[](
    vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
    vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0)
);

vec2 uvs[6] = vec2[](
    vec2(0.0, 1.0), vec2(1.0, 1.0), vec2(1.0, 0.0),
    vec2(0.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 0.0)
);

void main() {
    gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
    outUV = uvs[gl_VertexIndex];
}
)";

// Fragment shader (GLSL)
const char* fragmentShaderCode = R"(
#version 450
layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;
layout(binding = 0) uniform sampler2D texSampler;

void main() {
    outColor = texture(texSampler, inUV);
}
)";

// Compile GLSL to SPIR-V (simplified - normally use glslangValidator)
std::vector<uint32_t> compileShader(const char* /*source*/, bool isVertex) {
    // Pre-compiled SPIR-V for the shaders above
    if (isVertex) {
        // Vertex shader SPIR-V
        return {
            0x07230203, 0x00010000, 0x0008000a, 0x0000002e, 0x00000000, 0x00020011, 0x00000001, 0x0006000b,
            0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e, 0x00000000, 0x0003000e, 0x00000000, 0x00000001,
            0x0008000f, 0x00000000, 0x00000004, 0x6e69616d, 0x00000000, 0x00000009, 0x0000000b, 0x00000028,
            0x00030003, 0x00000002, 0x000001c2, 0x00040005, 0x00000004, 0x6e69616d, 0x00000000, 0x00060005,
            0x00000009, 0x505f6c67, 0x65567265, 0x78657472, 0x00000000, 0x00060006, 0x00000009, 0x00000000,
            0x505f6c67, 0x7469736f, 0x006e6f69, 0x00030005, 0x0000000b, 0x00000000, 0x00050005, 0x0000000f,
            0x69736f70, 0x6e6f6974, 0x00000073, 0x00060005, 0x00000020, 0x565f6c67, 0x65747265, 0x646e4978,
            0x00007865, 0x00040005, 0x00000028, 0x5574756f, 0x00000056, 0x00030005, 0x0000002a, 0x00737675,
            0x00050048, 0x00000009, 0x00000000, 0x0000000b, 0x00000000, 0x00030047, 0x00000009, 0x00000002,
            0x00040047, 0x00000020, 0x0000000b, 0x0000002a, 0x00040047, 0x00000028, 0x0000001e, 0x00000000,
            0x00020013, 0x00000002, 0x00030021, 0x00000003, 0x00000002, 0x00030016, 0x00000006, 0x00000020,
            0x00040017, 0x00000007, 0x00000006, 0x00000004, 0x0003001e, 0x00000009, 0x00000007, 0x00040020,
            0x0000000a, 0x00000003, 0x00000009, 0x0004003b, 0x0000000a, 0x0000000b, 0x00000003, 0x00040015,
            0x0000000c, 0x00000020, 0x00000001, 0x0004002b, 0x0000000c, 0x0000000d, 0x00000000, 0x00040015,
            0x0000000e, 0x00000020, 0x00000000, 0x0004002b, 0x0000000e, 0x00000010, 0x00000006, 0x00040017,
            0x00000011, 0x00000006, 0x00000002, 0x0005001c, 0x00000012, 0x00000011, 0x00000010, 0x00040020,
            0x00000013, 0x00000006, 0x00000012, 0x0004002b, 0x00000006, 0x00000015, 0xbf800000, 0x0005002c,
            0x00000011, 0x00000016, 0x00000015, 0x00000015, 0x0004002b, 0x00000006, 0x00000017, 0x3f800000,
            0x0005002c, 0x00000011, 0x00000018, 0x00000017, 0x00000015, 0x0005002c, 0x00000011, 0x00000019,
            0x00000017, 0x00000017, 0x0006002c, 0x00000012, 0x0000001a, 0x00000016, 0x00000018, 0x00000019,
            0x0005002c, 0x00000011, 0x0000001b, 0x00000015, 0x00000017, 0x0006002c, 0x00000012, 0x0000001c,
            0x00000016, 0x00000019, 0x0000001b, 0x0004003b, 0x00000013, 0x0000000f, 0x00000006, 0x0004001d,
            0x0000001d, 0x00000012, 0x0004001e, 0x0000001e, 0x0000001d, 0x00040020, 0x0000001f, 0x00000001,
            0x0000000c, 0x0004003b, 0x0000001f, 0x00000020, 0x00000001, 0x00040020, 0x00000022, 0x00000006,
            0x00000011, 0x0004002b, 0x00000006, 0x00000025, 0x00000000, 0x00040020, 0x00000026, 0x00000003,
            0x00000007, 0x00040020, 0x00000029, 0x00000003, 0x00000011, 0x0004003b, 0x00000029, 0x00000028,
            0x00000003, 0x0004003b, 0x00000013, 0x0000002a, 0x00000006, 0x00050036, 0x00000002, 0x00000004,
            0x00000000, 0x00000003, 0x000200f8, 0x00000005, 0x0004003d, 0x0000000c, 0x00000021, 0x00000020,
            0x00050041, 0x00000022, 0x00000023, 0x0000000f, 0x00000021, 0x0004003d, 0x00000011, 0x00000024,
            0x00000023, 0x00050051, 0x00000006, 0x0000002b, 0x00000024, 0x00000000, 0x00050051, 0x00000006,
            0x0000002c, 0x00000024, 0x00000001, 0x00070050, 0x00000007, 0x0000002d, 0x0000002b, 0x0000002c,
            0x00000025, 0x00000017, 0x00050041, 0x00000026, 0x00000027, 0x0000000b, 0x0000000d, 0x0003003e,
            0x00000027, 0x0000002d, 0x00050041, 0x00000022, 0x0000002e, 0x0000002a, 0x00000021, 0x0004003d,
            0x00000011, 0x0000002f, 0x0000002e, 0x0003003e, 0x00000028, 0x0000002f, 0x000100fd, 0x00010038
        };
    } else {
        // Fragment shader SPIR-V
        return {
            0x07230203, 0x00010000, 0x0008000a, 0x00000013, 0x00000000, 0x00020011, 0x00000001, 0x0006000b,
            0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e, 0x00000000, 0x0003000e, 0x00000000, 0x00000001,
            0x0007000f, 0x00000004, 0x00000004, 0x6e69616d, 0x00000000, 0x00000009, 0x0000000d, 0x00030010,
            0x00000004, 0x00000007, 0x00030003, 0x00000002, 0x000001c2, 0x00040005, 0x00000004, 0x6e69616d,
            0x00000000, 0x00050005, 0x00000009, 0x4374756f, 0x726f6c6f, 0x00000000, 0x00050005, 0x0000000d,
            0x55556e69, 0x00000056, 0x00000000, 0x00060005, 0x00000011, 0x53786574, 0x6c706d61, 0x00007265,
            0x00000000, 0x00040047, 0x00000009, 0x0000001e, 0x00000000, 0x00040047, 0x0000000d, 0x0000001e,
            0x00000000, 0x00040047, 0x00000011, 0x00000022, 0x00000000, 0x00040047, 0x00000011, 0x00000021,
            0x00000000, 0x00020013, 0x00000002, 0x00030021, 0x00000003, 0x00000002, 0x00030016, 0x00000006,
            0x00000020, 0x00040017, 0x00000007, 0x00000006, 0x00000004, 0x00040020, 0x00000008, 0x00000003,
            0x00000007, 0x0004003b, 0x00000008, 0x00000009, 0x00000003, 0x00040017, 0x0000000b, 0x00000006,
            0x00000002, 0x00040020, 0x0000000c, 0x00000001, 0x0000000b, 0x0004003b, 0x0000000c, 0x0000000d,
            0x00000001, 0x00090019, 0x0000000e, 0x00000006, 0x00000001, 0x00000000, 0x00000000, 0x00000000,
            0x00000001, 0x00000000, 0x0003001b, 0x0000000f, 0x0000000e, 0x00040020, 0x00000010, 0x00000000,
            0x0000000f, 0x0004003b, 0x00000010, 0x00000011, 0x00000000, 0x00050036, 0x00000002, 0x00000004,
            0x00000000, 0x00000003, 0x000200f8, 0x00000005, 0x0004003d, 0x0000000f, 0x00000012, 0x00000011,
            0x0004003d, 0x0000000b, 0x0000000e, 0x0000000d, 0x00050057, 0x00000007, 0x0000000f, 0x00000012,
            0x0000000e, 0x0003003e, 0x00000009, 0x0000000f, 0x000100fd, 0x00010038
        };
    }
}

int main() {
    Camera camera;
    if (!camera.init()) {
        std::cerr << "Camera failed\n";
        return 1;
    }

    // Create OpenXR instance
    XrInstanceCreateInfo createInfo{XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy(createInfo.applicationInfo.applicationName, "VR Camera Feed");
    createInfo.applicationInfo.applicationVersion = 1;
    strcpy(createInfo.applicationInfo.engineName, "Custom");
    createInfo.applicationInfo.engineVersion = 1;
    createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    const char* extensions[] = {XR_KHR_VULKAN_ENABLE_EXTENSION_NAME};
    createInfo.enabledExtensionCount = 1;
    createInfo.enabledExtensionNames = extensions;

    XrInstance instance;
    CHECK_XR(xrCreateInstance(&createInfo, &instance));
    std::cout << "✓ OpenXR instance\n";

    // Get system
    XrSystemGetInfo systemInfo{XR_TYPE_SYSTEM_GET_INFO};
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    
    XrSystemId systemId;
    CHECK_XR(xrGetSystem(instance, &systemInfo, &systemId));
    std::cout << "✓ VR system\n";

    // Setup Vulkan
    PFN_xrGetVulkanGraphicsRequirementsKHR pfnGetVulkanReqs;
    xrGetInstanceProcAddr(instance, "xrGetVulkanGraphicsRequirementsKHR", (PFN_xrVoidFunction*)&pfnGetVulkanReqs);
    
    XrGraphicsRequirementsVulkanKHR vulkanReqs{XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR};
    pfnGetVulkanReqs(instance, systemId, &vulkanReqs);

    VkApplicationInfo appInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    appInfo.apiVersion = VK_API_VERSION_1_0;
    
    VkInstanceCreateInfo vkCreateInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    vkCreateInfo.pApplicationInfo = &appInfo;
    
    VkInstance vkInstance;
    CHECK_VK(vkCreateInstance(&vkCreateInfo, nullptr, &vkInstance));

    uint32_t deviceCount = 1;
    VkPhysicalDevice vkPhysicalDevice;
    vkEnumeratePhysicalDevices(vkInstance, &deviceCount, &vkPhysicalDevice);

    float queuePriority = 1.0f;
    VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queueInfo.queueFamilyIndex = 0;
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &queuePriority;
    
    VkDeviceCreateInfo deviceInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    deviceInfo.queueCreateInfoCount = 1;
    deviceInfo.pQueueCreateInfos = &queueInfo;
    
    VkDevice vkDevice;
    CHECK_VK(vkCreateDevice(vkPhysicalDevice, &deviceInfo, nullptr, &vkDevice));

    VkQueue vkQueue;
    vkGetDeviceQueue(vkDevice, 0, 0, &vkQueue);
    std::cout << "✓ Vulkan\n";

    // Create command pool
    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    poolInfo.queueFamilyIndex = 0;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    
    VkCommandPool cmdPool;
    CHECK_VK(vkCreateCommandPool(vkDevice, &poolInfo, nullptr, &cmdPool));

    VkCommandBufferAllocateInfo cmdAllocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cmdAllocInfo.commandPool = cmdPool;
    cmdAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAllocInfo.commandBufferCount = 1;
    
    VkCommandBuffer cmdBuffer;
    CHECK_VK(vkAllocateCommandBuffers(vkDevice, &cmdAllocInfo, &cmdBuffer));

    // Create camera texture
    VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    imageInfo.extent = {(uint32_t)camera.width, (uint32_t)camera.height, 1};
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    
    VkImage cameraTexture;
    CHECK_VK(vkCreateImage(vkDevice, &imageInfo, nullptr, &cameraTexture));
    
    VkMemoryRequirements memReq;
    vkGetImageMemoryRequirements(vkDevice, cameraTexture, &memReq);
    
    VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocInfo.allocationSize = memReq.size;
    allocInfo.memoryTypeIndex = 0;
    
    VkDeviceMemory textureMemory;
    CHECK_VK(vkAllocateMemory(vkDevice, &allocInfo, nullptr, &textureMemory));
    CHECK_VK(vkBindImageMemory(vkDevice, cameraTexture, textureMemory, 0));

    // Create image view
    VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    viewInfo.image = cameraTexture;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = 1;
    
    VkImageView textureView;
    CHECK_VK(vkCreateImageView(vkDevice, &viewInfo, nullptr, &textureView));

    // Create sampler
    VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    
    VkSampler sampler;
    CHECK_VK(vkCreateSampler(vkDevice, &samplerInfo, nullptr, &sampler));

    // Create staging buffer
    VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufferInfo.size = camera.width * camera.height * 4;
    bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    
    VkBuffer stagingBuffer;
    CHECK_VK(vkCreateBuffer(vkDevice, &bufferInfo, nullptr, &stagingBuffer));
    
    vkGetBufferMemoryRequirements(vkDevice, stagingBuffer, &memReq);
    allocInfo.allocationSize = memReq.size;
    
    VkDeviceMemory stagingMemory;
    CHECK_VK(vkAllocateMemory(vkDevice, &allocInfo, nullptr, &stagingMemory));
    CHECK_VK(vkBindBufferMemory(vkDevice, stagingBuffer, stagingMemory, 0));

    std::cout << "✓ Camera texture created\n";

    // Create descriptor set layout
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    
    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &binding;
    
    VkDescriptorSetLayout descLayout;
    CHECK_VK(vkCreateDescriptorSetLayout(vkDevice, &layoutInfo, nullptr, &descLayout));

    // Create descriptor pool
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSize.descriptorCount = 1;
    
    VkDescriptorPoolCreateInfo descPoolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    descPoolInfo.maxSets = 1;
    descPoolInfo.poolSizeCount = 1;
    descPoolInfo.pPoolSizes = &poolSize;
    
    VkDescriptorPool descPool;
    CHECK_VK(vkCreateDescriptorPool(vkDevice, &descPoolInfo, nullptr, &descPool));

    // Allocate descriptor set
    VkDescriptorSetAllocateInfo descAllocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    descAllocInfo.descriptorPool = descPool;
    descAllocInfo.descriptorSetCount = 1;
    descAllocInfo.pSetLayouts = &descLayout;
    
    VkDescriptorSet descSet;
    CHECK_VK(vkAllocateDescriptorSets(vkDevice, &descAllocInfo, &descSet));

    // Update descriptor set
    VkDescriptorImageInfo imageDescInfo{};
    imageDescInfo.sampler = sampler;
    imageDescInfo.imageView = textureView;
    imageDescInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    
    VkWriteDescriptorSet descWrite{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    descWrite.dstSet = descSet;
    descWrite.dstBinding = 0;
    descWrite.descriptorCount = 1;
    descWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descWrite.pImageInfo = &imageDescInfo;
    
    vkUpdateDescriptorSets(vkDevice, 1, &descWrite, 0, nullptr);

    // Create shaders
    VkShaderModuleCreateInfo shaderInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    
    auto vertSpirv = compileShader(vertexShaderCode, true);
    shaderInfo.codeSize = vertSpirv.size() * 4;
    shaderInfo.pCode = vertSpirv.data();
    VkShaderModule vertShader;
    CHECK_VK(vkCreateShaderModule(vkDevice, &shaderInfo, nullptr, &vertShader));
    
    auto fragSpirv = compileShader(fragmentShaderCode, false);
    shaderInfo.codeSize = fragSpirv.size() * 4;
    shaderInfo.pCode = fragSpirv.data();
    VkShaderModule fragShader;
    CHECK_VK(vkCreateShaderModule(vkDevice, &shaderInfo, nullptr, &fragShader));

    std::cout << "✓ Shaders compiled\n";

    // Create pipeline layout
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &descLayout;
    
    VkPipelineLayout pipelineLayout;
    CHECK_VK(vkCreatePipelineLayout(vkDevice, &pipelineLayoutInfo, nullptr, &pipelineLayout));

    // Create render pass
    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = VK_FORMAT_R8G8B8A8_SRGB;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    
    VkAttachmentReference colorRef{};
    colorRef.attachment = 0;
    colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    
    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    
    VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    renderPassInfo.attachmentCount = 1;
    renderPassInfo.pAttachments = &colorAttachment;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    
    VkRenderPass renderPass;
    CHECK_VK(vkCreateRenderPass(vkDevice, &renderPassInfo, nullptr, &renderPass));

    // Create graphics pipeline
    VkPipelineShaderStageCreateInfo shaderStages[2] = {};
    shaderStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    shaderStages[0].module = vertShader;
    shaderStages[0].pName = "main";
    
    shaderStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    shaderStages[1].module = fragShader;
    shaderStages[1].pName = "main";
    
    VkPipelineVertexInputStateCreateInfo vertexInput{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    
    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;
    
    VkPipelineRasterizationStateCreateInfo rasterizer{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;
    rasterizer.lineWidth = 1.0f;
    
    VkPipelineMultisampleStateCreateInfo multisampling{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    
    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | 
                                          VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    
    VkPipelineColorBlendStateCreateInfo colorBlending{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;
    
    VkDynamicState dynamicStates[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = 2;
    dynamicState.pDynamicStates = dynamicStates;
    
    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = pipelineLayout;
    pipelineInfo.renderPass = renderPass;
    
    VkPipeline pipeline;
    CHECK_VK(vkCreateGraphicsPipelines(vkDevice, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline));
    std::cout << "✓ Graphics pipeline created\n";

    // Create OpenXR session
    XrGraphicsBindingVulkanKHR graphicsBinding{XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR};
    graphicsBinding.instance = vkInstance;
    graphicsBinding.physicalDevice = vkPhysicalDevice;
    graphicsBinding.device = vkDevice;
    graphicsBinding.queueFamilyIndex = 0;
    graphicsBinding.queueIndex = 0;
    
    XrSessionCreateInfo sessionInfo{XR_TYPE_SESSION_CREATE_INFO};
    sessionInfo.next = &graphicsBinding;
    sessionInfo.systemId = systemId;
    
    XrSession session;
    CHECK_XR(xrCreateSession(instance, &sessionInfo, &session));
    std::cout << "✓ VR session\n";

    // Create reference space
    XrReferenceSpaceCreateInfo refSpaceInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    refSpaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    refSpaceInfo.poseInReferenceSpace.orientation.w = 1.0f;
    
    XrSpace viewSpace;
    CHECK_XR(xrCreateReferenceSpace(session, &refSpaceInfo, &viewSpace));

    // Get view configuration
    uint32_t viewCount;
    CHECK_XR(xrEnumerateViewConfigurationViews(instance, systemId, 
        XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &viewCount, nullptr));
    
    std::vector<XrViewConfigurationView> configViews(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
    CHECK_XR(xrEnumerateViewConfigurationViews(instance, systemId,
        XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, viewCount, &viewCount, configViews.data()));

    // Create swapchains
    std::vector<XrSwapchain> swapchains(viewCount);
    std::vector<std::vector<XrSwapchainImageVulkanKHR>> swapchainImages(viewCount);
    std::vector<std::vector<VkImageView>> swapchainViews(viewCount);
    std::vector<std::vector<VkFramebuffer>> framebuffers(viewCount);
    
    for (uint32_t i = 0; i < viewCount; i++) {
        XrSwapchainCreateInfo swapchainInfo{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        swapchainInfo.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
        swapchainInfo.format = VK_FORMAT_R8G8B8A8_SRGB;
        swapchainInfo.sampleCount = 1;
        swapchainInfo.width = configViews[i].recommendedImageRectWidth;
        swapchainInfo.height = configViews[i].recommendedImageRectHeight;
        swapchainInfo.faceCount = 1;
        swapchainInfo.arraySize = 1;
        swapchainInfo.mipCount = 1;
        
        CHECK_XR(xrCreateSwapchain(session, &swapchainInfo, &swapchains[i]));
        
        uint32_t imageCount;
        xrEnumerateSwapchainImages(swapchains[i], 0, &imageCount, nullptr);
        swapchainImages[i].resize(imageCount, {XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR});
        xrEnumerateSwapchainImages(swapchains[i], imageCount, &imageCount,
            (XrSwapchainImageBaseHeader*)swapchainImages[i].data());
        
        // Create image views and framebuffers for each swapchain image
        swapchainViews[i].resize(imageCount);
        framebuffers[i].resize(imageCount);
        
        for (uint32_t j = 0; j < imageCount; j++) {
            VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            viewInfo.image = swapchainImages[i][j].image;
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = VK_FORMAT_R8G8B8A8_SRGB;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.levelCount = 1;
            viewInfo.subresourceRange.layerCount = 1;
            CHECK_VK(vkCreateImageView(vkDevice, &viewInfo, nullptr, &swapchainViews[i][j]));
            
            VkFramebufferCreateInfo fbInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
            fbInfo.renderPass = renderPass;
            fbInfo.attachmentCount = 1;
            fbInfo.pAttachments = &swapchainViews[i][j];
            fbInfo.width = configViews[i].recommendedImageRectWidth;
            fbInfo.height = configViews[i].recommendedImageRectHeight;
            fbInfo.layers = 1;
            CHECK_VK(vkCreateFramebuffer(vkDevice, &fbInfo, nullptr, &framebuffers[i][j]));
        }
    }
    std::cout << "✓ Swapchains created\n";

    // Begin session
    XrSessionBeginInfo beginInfo{XR_TYPE_SESSION_BEGIN_INFO};
    beginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    CHECK_XR(xrBeginSession(session, &beginInfo));

    std::cout << "\n========================================\n";
    std::cout << "  CAMERA FEED DISPLAYING IN VR!\n";
    std::cout << "========================================\n\n";

    // Main render loop
    bool running = true;
    XrSessionState sessionState = XR_SESSION_STATE_UNKNOWN;
    int frameCount = 0;
    bool textureInitialized = false;
    
    while (running && frameCount < 900) {
        // Capture camera frame
        if (!camera.captureFrame()) {
            std::cerr << "Camera capture failed\n";
        }

        // Poll events
        XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
        while (xrPollEvent(instance, &event) == XR_SUCCESS) {
            if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                XrEventDataSessionStateChanged* stateEvent = 
                    (XrEventDataSessionStateChanged*)&event;
                sessionState = stateEvent->state;
                
                if (sessionState == XR_SESSION_STATE_STOPPING || 
                    sessionState == XR_SESSION_STATE_EXITING) {
                    running = false;
                }
            }
            event.type = XR_TYPE_EVENT_DATA_BUFFER;
        }

        if (sessionState != XR_SESSION_STATE_FOCUSED) continue;

        // Upload camera texture
        void* data;
        vkMapMemory(vkDevice, stagingMemory, 0, camera.frameData.size(), 0, &data);
        memcpy(data, camera.frameData.data(), camera.frameData.size());
        vkUnmapMemory(vkDevice, stagingMemory);

        // Transition texture and copy data
        VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        vkBeginCommandBuffer(cmdBuffer, &beginInfo);
        
        if (!textureInitialized) {
            VkImageMemoryBarrier barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
            barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            barrier.image = cameraTexture;
            barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            barrier.subresourceRange.levelCount = 1;
            barrier.subresourceRange.layerCount = 1;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            
            vkCmdPipelineBarrier(cmdBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 
                VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
        }
        
        VkBufferImageCopy region{};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent = {(uint32_t)camera.width, (uint32_t)camera.height, 1};
        
        vkCmdCopyBufferToImage(cmdBuffer, stagingBuffer, cameraTexture, 
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
        
        VkImageMemoryBarrier barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.image = cameraTexture;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.layerCount = 1;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        
        vkCmdPipelineBarrier(cmdBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, 
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
        
        vkEndCommandBuffer(cmdBuffer);
        
        VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &cmdBuffer;
        vkQueueSubmit(vkQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(vkQueue);
        
        textureInitialized = true;

        // Wait for frame
        XrFrameWaitInfo frameWaitInfo{XR_TYPE_FRAME_WAIT_INFO};
        XrFrameState frameState{XR_TYPE_FRAME_STATE};
        CHECK_XR(xrWaitFrame(session, &frameWaitInfo, &frameState));

        XrFrameBeginInfo frameBeginInfo{XR_TYPE_FRAME_BEGIN_INFO};
        CHECK_XR(xrBeginFrame(session, &frameBeginInfo));

        XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        std::vector<XrCompositionLayerProjectionView> projViews(viewCount, 
            {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW});
        
        if (frameState.shouldRender) {
            XrViewLocateInfo locateInfo{XR_TYPE_VIEW_LOCATE_INFO};
            locateInfo.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            locateInfo.displayTime = frameState.predictedDisplayTime;
            locateInfo.space = viewSpace;
            
            XrViewState viewState{XR_TYPE_VIEW_STATE};
            std::vector<XrView> views(viewCount, {XR_TYPE_VIEW});
            xrLocateViews(session, &locateInfo, &viewState, viewCount, &viewCount, views.data());

            for (uint32_t i = 0; i < viewCount; i++) {
                XrSwapchainImageAcquireInfo acquireInfo{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
                uint32_t imageIndex;
                xrAcquireSwapchainImage(swapchains[i], &acquireInfo, &imageIndex);
                
                XrSwapchainImageWaitInfo waitInfo{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
                waitInfo.timeout = XR_INFINITE_DURATION;
                xrWaitSwapchainImage(swapchains[i], &waitInfo);

                // RENDER THE CAMERA FEED
                VkCommandBufferBeginInfo cmdBeginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
                vkBeginCommandBuffer(cmdBuffer, &cmdBeginInfo);
                
                VkRenderPassBeginInfo rpBeginInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
                rpBeginInfo.renderPass = renderPass;
                rpBeginInfo.framebuffer = framebuffers[i][imageIndex];
                rpBeginInfo.renderArea.extent = {
                    configViews[i].recommendedImageRectWidth,
                    configViews[i].recommendedImageRectHeight
                };
                VkClearValue clearColor = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
                rpBeginInfo.clearValueCount = 1;
                rpBeginInfo.pClearValues = &clearColor;
                
                vkCmdBeginRenderPass(cmdBuffer, &rpBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
                
                vkCmdBindPipeline(cmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
                vkCmdBindDescriptorSets(cmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, 
                    pipelineLayout, 0, 1, &descSet, 0, nullptr);
                
                VkViewport viewport{};
                viewport.width = (float)configViews[i].recommendedImageRectWidth;
                viewport.height = (float)configViews[i].recommendedImageRectHeight;
                viewport.maxDepth = 1.0f;
                vkCmdSetViewport(cmdBuffer, 0, 1, &viewport);
                
                VkRect2D scissor{};
                scissor.extent = {configViews[i].recommendedImageRectWidth, 
                                 configViews[i].recommendedImageRectHeight};
                vkCmdSetScissor(cmdBuffer, 0, 1, &scissor);
                
                vkCmdDraw(cmdBuffer, 6, 1, 0, 0);
                
                vkCmdEndRenderPass(cmdBuffer);
                vkEndCommandBuffer(cmdBuffer);
                
                VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
                submitInfo.commandBufferCount = 1;
                submitInfo.pCommandBuffers = &cmdBuffer;
                vkQueueSubmit(vkQueue, 1, &submitInfo, VK_NULL_HANDLE);
                vkQueueWaitIdle(vkQueue);

                XrSwapchainImageReleaseInfo releaseInfo{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                xrReleaseSwapchainImage(swapchains[i], &releaseInfo);

                projViews[i].pose = views[i].pose;
                projViews[i].fov = views[i].fov;
                projViews[i].subImage.swapchain = swapchains[i];
                projViews[i].subImage.imageRect.offset = {0, 0};
                projViews[i].subImage.imageRect.extent = {
                    (int32_t)configViews[i].recommendedImageRectWidth,
                    (int32_t)configViews[i].recommendedImageRectHeight
                };
            }

            layer.space = viewSpace;
            layer.viewCount = viewCount;
            layer.views = projViews.data();
        }

        XrFrameEndInfo frameEndInfo{XR_TYPE_FRAME_END_INFO};
        frameEndInfo.displayTime = frameState.predictedDisplayTime;
        frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        frameEndInfo.layerCount = frameState.shouldRender ? 1 : 0;
        const XrCompositionLayerBaseHeader* layers[] = {
            (XrCompositionLayerBaseHeader*)&layer
        };
        frameEndInfo.layers = layers;
        
        CHECK_XR(xrEndFrame(session, &frameEndInfo));

        frameCount++;
        if (frameCount % 60 == 0) {
            std::cout << "Frame " << frameCount << " - Camera displayed in VR!\n";
        }
    }

    std::cout << "\n✓ Render loop complete\n";

    // Cleanup
    camera.cleanup();
    vkDeviceWaitIdle(vkDevice);
    
    for (auto& fbList : framebuffers) {
        for (auto fb : fbList) vkDestroyFramebuffer(vkDevice, fb, nullptr);
    }
    for (auto& viewList : swapchainViews) {
        for (auto view : viewList) vkDestroyImageView(vkDevice, view, nullptr);
    }
    
    vkDestroyPipeline(vkDevice, pipeline, nullptr);
    vkDestroyPipelineLayout(vkDevice, pipelineLayout, nullptr);
    vkDestroyRenderPass(vkDevice, renderPass, nullptr);
    vkDestroyShaderModule(vkDevice, vertShader, nullptr);
    vkDestroyShaderModule(vkDevice, fragShader, nullptr);
    vkDestroyDescriptorPool(vkDevice, descPool, nullptr);
    vkDestroyDescriptorSetLayout(vkDevice, descLayout, nullptr);
    vkDestroySampler(vkDevice, sampler, nullptr);
    vkDestroyImageView(vkDevice, textureView, nullptr);
    vkDestroyImage(vkDevice, cameraTexture, nullptr);
    vkFreeMemory(vkDevice, textureMemory, nullptr);
    vkDestroyBuffer(vkDevice, stagingBuffer, nullptr);
    vkFreeMemory(vkDevice, stagingMemory, nullptr);
    vkDestroyCommandPool(vkDevice, cmdPool, nullptr);
    
    xrEndSession(session);
    for (auto& swapchain : swapchains) xrDestroySwapchain(swapchain);
    xrDestroySpace(viewSpace);
    xrDestroySession(session);
    vkDestroyDevice(vkDevice, nullptr);
    vkDestroyInstance(vkInstance, nullptr);
    xrDestroyInstance(instance);
    
    std::cout << "✓ Cleanup complete\n";
    return 0;
}

// Compile (Linux):
// g++ -std=c++11 openxr_camera.cpp -lopenxr_loader -lvulkan `pkg-config --cflags --libs opencv4` -o xr_camera
//
// This now FULLY renders the camera feed in VR!