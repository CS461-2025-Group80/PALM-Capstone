1. Create an AI pilot capable of autonomously navigating a college campus parking environment.
    This is a large goal of the project and it is good to keep it in mind as we write other, smaller requirements that will help us achieve this.
    Priority: Medium
2. Train or used a pre-trained an artificial intelligence model to classify parking violations.
    This model will be the brain of the core/advertised functionality of the robot (detecting parking violations) and a pre-trained vision model will likely be time-saving if we augment our own data.
    Priority: Medium
3. Collect a large amount of raw image data by manually driving the robot around Oregon State's campus parking.
     Raw image data will be required to train a model from scratch or give a pretrained model context of what our robot should achieve; no matter what, a large amount of labeled data is needed.
     Priority: Medium
4. Create an automatic data collection pipeline that takes images and uploads them to Azure Cloud BLOB storage via ROS2.
     Taking images via the robot will ensure that our training data matches what the robot sees during inference, but storing this much training data on the robot is not possible - uploading it to Azure cloud quickly and     routinely is a required part of data collection in this project.
     Priority: High
5. Label training data parking violations.
    Raw images of parking violations (or proper parking) are not useful in model training; we need to label the training images so that the model has something to "look for".
     Priority: High
6. Validate and test the AI pilot in both simulated and real-world campus environments.
    Testing the AI pilot, including its driving capabilities and inference are crucial to understanding the robots weaknesses and being able to iterate to improve.
     Priority: High
7. Ensure that all communication between the robot and the server is secure and respects data privacy constraints.
    Keeping sensitive data such as license plates and inevitable images of pedestrians is crucial in maintaining trust on a university campus and complying with privacy laws.
     Priority: High
8. Document all system components, APIs, and workflows for future continuation of the project by the REVOBOTS organization and/or future capstone teams.
    REVOBOTS is an organization that will continue development after our capstone completes, so allowing them to continue our development is important for their company.
     Priority: High
9. Create a communication module that reports detected violations, including image evidence and metadata, to the REVOBOTS cloud service
    The robot should never store sensitive data locally - keeping the collected and inferred data in a secure database is important for storage reasons and also privacy.
     Priority: Medium
10. Structure BLOB uploads in a way that is effective for model training and ease of use including proper containerization and metadata.
    Uploading unstructured image data or useless metadata will make training less efective, increase storage requirements, and take longer.
     Priority: High


