PALM CS Capstone, Group 80  
Luke Hashbarger, Andrew Fief, Justice Peyton  
CS 461 001 F2025  
Requirements  


## Problem Statement & Scope

The project PALM project entails creating and training an AI pilot for use in the OS of a REVOBOTS robot. The AI pilot will travel on college campuses and travel to parking lots and spots and will correctly identify parking violations and report them to a REVOBOTS server that will communicate with the college's parking enforcement. Additionally, the AI should be capable of using its robotic arm to deliver and move packages on-campus.

### Users

#### Primary Users

A primary user of the PALM project will be the college campus parking enforcement organization/group. This organization will directly receive input from PALM-equipped robots regarding parking violations, including those vehicles' license plates and time of the violation, of which they can take action on.

Another primary user of the PALM project will be on-campus delivery services that require the use of the PALM robot's arm.

#### Secondary Users

The secondary users of the PALM project will be the REVOBOTS organization itself. In the scenario of a PALM robot not being able to succeed in a particular task, REVOBOTS will be informed and provided remote pilotting capabilities to help the robot get to a more stable situation.

### Goals
These are goals that the project WILL fulfill.

1. Create an AI pilot that can autonomously navigate a college campus.
2. Enable the AI pilot to identify parking violations accurately.
3. Ensure the AI pilot can report identified violations to a REVOBOTS server.
4. Appropriately train the AI pilot to use its robotic arm for delivering on-campus packages.
5. Allow for easy passing-the-torch from human pilotting to AI pilotting and vice versa.

### Non-Goals
These are goals that the project does NOT fulfill.

1. Performing actual citations or legal reports on parking violation.
2. Creating and designing hardware for the project.
3. Designing code and functionality for scanning license plates.

## Consolidated Requirement
For high-priority items, include acceptance criteria directly under the item

### Non Functional Requirements
  **Performance:** Real-time obstacle avoidance with low latency. High accuracy and precision in detecting parking violations.
  
  **Reliability:** The system must operate continuously for multiple hours. Sensitive data is secured in a reliable cloud storage provider.
  
  **Scalability:** Cloud-based infrastructure should handle logs and image uploads from multiple robots.
  
  **Privacy:** All data must remain confidential. Training data remains anonymous. Code is under MIT license.
  
  **Compliance:** Must adhere to OSU Capstone academic integrity guidelines and REVOBOTS confidentiality agreements. REVOBOTS must comply with Oregon State University (and future campus') privacy agreements
  
## Prioritization Method & Results
Requirements are prioritized based on their impact on system safety, reliability, and project continuity.

**Urgent Priority** requirements address privacy, digital safety, and physical safety concerns. These include data integrity, object detection and avoidance, and data security; areas where failure could result in harm, data loss, or violations of privacy.

**High Priority** requirements support the development process and team collaboration. This includes maintaining effective communication between teams, ensuring proper documentation, and enabling seamless integration across components.

**Medium Priority** requirements focus on the core functionality of the project, such as implementing a working AI model, ensuring proper camera functionality, and achieving essential system operations necessary for the PALM robot’s purpose.

**Low Priority** requirements involve enhancements and refinements that improve performance or usability but are not critical to the project’s success. Examples include optimizing API accessibility, improving data labeling accuracy, or polishing secondary features.

## Ethics, Risks, and Constraints
### Ethical Considerations
  **Privacy:** All captured images and data will be stored securely, accessable only by authorized personnel. No public sharing of identifiable data.

  The REVOBOTS codebase is currently under MIT licensure.
  
  **Transparency:** Data used for AI model training will be labeled and processed responsibly.
### Risks
  **Technical Risk:** Integration challenges between ROS2 and Azure BLOB services
  
  **Operational Risk:** Libability issues when testing a robot with sub-standard object detection and collision avoidance.
  
  **Ethical Risk:** Handling and storing identifiable information including pedestrian faces, license plates, and vehicle images.
### Constraints
  Confidentiality of REVOBOTS proprietary property.
  
  Limited access to physical robots during testing periods and code drafting stages
  
## Merge Methodology & Change References
asdf 

## Appendix
### Andrew's Individal Requirements:
**1. Create an AI pilot capable of autonomously navigating a college campus parking environment.**

This is a large goal of the project and it is good to keep it in mind as we write other, smaller requirements that will help us achieve this.

Priority: Medium

**2. Train or used a pre-trained an artificial intelligence model to classify parking violations.**

This model will be the brain of the core/advertised functionality of the robot (detecting parking violations) and a pre-trained vision model will likely be time-saving if we augment our own data.

Priority: Medium

**3. Collect a large amount of raw image data by manually driving the robot around Oregon State's campus parking.**

Raw image data will be required to train a model from scratch or give a pretrained model context of what our robot should achieve; no matter what, a large amount of labeled data is needed.

Priority: Medium

**4. Create an automatic data collection pipeline that takes images and uploads them to Azure Cloud BLOB storage via ROS2.**

Taking images via the robot will ensure that our training data matches what the robot sees during inference, but storing this much training data on the robot is not possible - uploading it to Azure cloud quickly and     routinely is a required part of data collection in this project.

Priority: High

**5. Label training data parking violations.**

Raw images of parking violations (or proper parking) are not useful in model training; we need to label the training images so that the model has something to "look for".

Priority: High

**6. Validate and test the AI pilot in both simulated and real-world campus environments.**

Testing the AI pilot, including its driving capabilities and inference are crucial to understanding the robots weaknesses and being able to iterate to improve.

Priority: High

**7. Ensure that all communication between the robot and the server is secure and respects data privacy constraints.**

Keeping sensitive data such as license plates and inevitable images of pedestrians is crucial in maintaining trust on a university campus and complying with privacy laws.

Priority: High

**8. Document all system components, APIs, and workflows for future continuation of the project by the REVOBOTS organization and/or future capstone teams.**

REVOBOTS is an organization that will continue development after our capstone completes, so allowing them to continue our development is important for their company.

Priority: High

**9. Create a communication module that reports detected violations, including image evidence and metadata, to the REVOBOTS cloud service**

The robot should never store sensitive data locally - keeping the collected and inferred data in a secure database is important for storage reasons and also privacy.

Priority: Medium

**10. Structure BLOB uploads in a way that is effective for model training and ease of use including proper containerization and metadata.**

Uploading unstructured image data or useless metadata will make training less efective, increase storage requirements, and take longer.

Priority: High


### Justice's Individual Requirements:
**1. Design API for our ROS2 environment for use by our team's AI**

The ROS2 system shall be capable of being interfaced with our team's AI pilot with a well-made API that can be called upon by the AI, allowing for a smooth and modular way of integrating the AI pilot with the robot's movement.

Priority: High

**2. Design a succinct logging system for the ROS2 AI API**

The ROS2 system shall keep an accurate logging of every API call that the AI pilot makes in order to keep clear track of everything that the AI does during operation.

Priority: High

**3. Maintain a clear and straightforward method of communication between the team and the project proposer**

To avoid future miscommunication or confusion, a clear and straightforward platform will be maintained and used for our team and our project proposer for effective communication on Discord.

Priority: High

**4. Keep a clean set of documentation for all API that we design for use by our team and future REVOBOTS developers**

The code that we make will be used by our team and future developers at REVOBOTS. We need to maintain a clean set of documentation for our own sanity and for future developers'.

Priority: High

**5. Keep ROS2 nodes as low-latency as possible**

ROS2 communicates with nodes-- it is a maximal priority to make sure that the nodes arne't doing anything too expensive and are correctly optimized to prioritize speed.

Priority: High

**6. Update and modify REVOBOTS's existing ROS2 architecture for speed and modularity**

This is not a major requirement, but it will be important to improve existing architecture in case that it is harming or slowing down our project, perhaps after the AI is fully implemented.

Priority: Low

**7. Maintain constant or immediate access to REVOBOTS hardware for team testing and execution**

The team needs immediate or near-immediate access to REVOBOTS hardware to test our code and functionalities, with this access needing to be constant throughout the project.

Priority: High

**8. Design imperative safety measures and boundaries for the PALM robot via LIDAR sensors**

LIDAR will be used to measure imperative distances and will operate independently of the AI, which will be used to restrict the robot from getting too close to anything.

Priority: High

**9. Provide the overall robot functionality for requesting a human pilot**

This can be used by the imperative safety measures or if the AI self-determines that it cannot complete a task.

Priority: High

**10. Maintain clear communication of ROS2 development with REVOBOTS and the team to prevent merge conflicts**

REVOBOTS will be developing alongside the PALM project, so it is imperative that our work doesn't collide and, in the case that it does, we communicate on how to proceed.

Priority: High


### Luke's Individual Requirements:

**1. Use Webxr and mediasoup to create a VR environment**

This can be used to allow greater understanding of the environment during teleoperations.

Priority: Medium

**2. Incorporate an Nvidia Jetson Processor for image streaming**

The Nvidia Jetson Processor can encode images quicker and allow for quicker streaming of images for teleoperations. 

Priority: Low

**3. Establish stable communication protocols for commands and video**

Vital for functional operation, commands and video needs to be reliably communicated between the robot and operator. 

Priority: High

**4. A user interface to control the robot during teleoperations**

A UI controlling the drivetrain and the arm's joints will be the control mechanism for the robot during teleoperations. 

Priority: Medium

**5. Streaming of all robot cameras to a video conference style environment**

This additional capability would allow more stakeholders to see how the robot is able to operate.

Priority: Low

**6. Create a system where the robot is able to signal it requires human intervention**

Creating a system for human control is useless unless it can transition from autonomous to human control in unfamiliar environments.

**7. Improve communication between Capstone team and other REVOBOTS developers**

In addition to the project proposer there are several other members working on REVOBOTS who are able to assist different members of the group and creating a mutually accessible location for communication between members and them could prove helpful.

Priority: Medium

**8. Create data collection pipelines during teleoperation**

Additional data pipelines during teleoperations can be used to train AI agents and improve their abilities in situations they struggle with. 

Priority: Medium

**9. Ensure all capabilities are well tested**

Testing will be vital to ensuring complete functionality and taking a test often approach can improve development cycles.

Priority: Medium

**10. Ensure readable and well structured code**

Future REVOBOTS developers will need well documented and understandable code in order to ensure continued development is possible. 

Priority: High



