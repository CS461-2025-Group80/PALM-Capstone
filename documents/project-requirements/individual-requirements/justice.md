Justice's 10 Individual Requirements

### 1. Design API for our ROS2 environment for use by our team's AI
The ROS2 system shall be capable of being interfaced with our team's AI pilot with a well-made API that can be called upon by the AI, allowing for a smooth and modular way of integrating the AI pilot with the robot's movement.

Priority: High

### 2. Design a succinct logging system for the ROS2 AI API
The ROS2 system shall keep an accurate logging of every API call that the AI pilot makes in order to keep clear track of everything that the AI does during operation.

Priority: High

### 3. Maintain a clear and straightforward method of communication between the team and the project proposer
To avoid future miscommunication or confusion, a clear and straightforward platform will be maintained and used for our team and our project proposer for effective communication on Discord.

Priority: High

### 4. Keep a clean set of documentation for all API that we design for use by our team and future REVOBOTS developers
The code that we make will be used by our team and future developers at REVOBOTS. We need to maintain a clean set of documentation for our own sanity and for future developers'.

Priority: High

### 5. Keep ROS2 nodes as low-latency as possible
ROS2 communicates with nodes-- it is a maximal priority to make sure that the nodes arne't doing anything too expensive and are correctly optimized to prioritize speed.

Priority: High

### 6. Update and modify REVOBOTS's existing ROS2 architecture for speed and modularity
This is not a major requirement, but it will be important to improve existing architecture in case that it is harming or slowing down our project, perhaps after the AI is fully implemented.

Priority: Low

### 7. Maintain constant or immediate access to REVOBOTS hardware for team testing and execution
The team needs immediate or near-immediate access to REVOBOTS hardware to test our code and functionalities, with this access needing to be constant throughout the project.

Priority: High

### 8. Design imperative safety measures and boundaries for the PALM robot via LIDAR sensors
LIDAR will be used to measure imperative distances and will operate independently of the AI, which will be used to restrict the robot from getting too close to anything.

Priority: High

### 9. Provide the overall robot functionality for requesting a human pilot 
This can be used by the imperative safety measures or if the AI self-determines that it cannot complete a task.

Priority: High

### 10. Maintain clear communication of ROS2 development with REVOBOTS and the team to prevent merge conflicts
REVOBOTS will be developing alongside the PALM project, so it is imperative that our work doesn't collide and, in the case that it does, we communicate on how to proceed.

Priority: High

