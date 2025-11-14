
# Questions
Hello @Rahul Khanna!

The Capstone project has requirements on reporting what we’re up to and the details of the project we’re working on. The questions here are intended to get answers that pertain to those requirements. Please let us know if there’s anything that we should redact or explicitly NOT mention to the Capstone course (in the case of intellectual property concerns).

1. Does the project have anybody additional that will be working with the robots beyond REVOBOTS pilots and engineers?

    - For example, will there be someone managing charging stations on-campus, working alongside the robots, etc?

2. What would you say are explicit measurements of success of the project?

    - For example, would that be if the robot maintains autonomy with little human interaction, if it successfully reports a violating car, if it can successfully scan license plates, etc?

3. Are there any major requirements or nonfunctional requirements that we should be aware of outside of what has already been told to the team?

4. Is there anything that we should be explicitly avoiding in the project?

    - Outside of basic criteria (such as avoiding getting hit by cars), is there anything in particular that we should avoid doing?

5. What would you say are the requirements of each team member?

    - Right now, we have Justice assigned to ROS2, Luke to Unity, and Andrew on BLOB. Is there any additional expectation or requirement for each team member?
    
6. Privacy of REVOBOTS's work is critical-- it's absolutely worth noting that we also were encouraged to use AI as much as we can to vibe-code. How would you like for us to use AI without exposing secret things of REVOBOTS?

7. Is the OSU posting of the PALM project up-to-date and something that we can reliably cite (https://eecs.engineering.oregonstate.edu/capstone/submission/pages/viewSingleProject.php?id=mYO0336z0Uyv9SkW)?

# Responses
***Please note that this was sent over Discord and thus is more of a conversation.***

Rahul Khanna

        1. Auto charging is not the part of project, although it is a nice to have. During development stages, we will perform manual charging.
        2. Auto navigation is the desired outcome. Auto License Plate reading and ticketing is already in testing stage and an independent component of the integrated solution.
        3. I don’t think of any additional major requirements
        4. I can’t think of anything like that.
        5. That is accurate. They need to start engaging with the respective development tutorials like Unity, ROS, online Azure training
        6. Privacy of communication, product details and core architecture has to be protected and only high level details can be made public. Somebody in REVOBOTS can do a check on documents before release
        7. Yes and you can cite as it is public

Justice

        Sounds good, thank you!

        For the 1st question, who might be some secondary users and primary users of the system? I gave some examples-- but I'm just curious to know if there's anybody in particular that will be specifically interacting with the robots (I know REVOBOTS pilots is definitely one of the primary users). Would you say that a user might be some REVOBOTS server that expects certain things of the robot? Perhaps a ticketing officer might be another user?

Rahul Khanna

        Primary users would be transportation department of schools and universities.
        Secondary users would be remote teleoperators that intervene or operate the AMR when not in autonomous mode or during trainable dataset recording
        Ticketing is an autonomous operation sland owned by the university IT department who in turn do it for transportation department

Justice

        I presume that this entails parsing a map, pathfinding, and using on-board sensors to travel and avoid collisions, then (similar to OSU's Starships)? 

Rahul Khanna
    
        Yes, we will use a combination of mapping and gen-ai based collision avoidance with traffic rules following. We will have a combination of LIDR and 3-D cameras that will be used to generate training data and learn how to react to a normal and not-normal situation
        We will use action-chunking transformers (ACT) to perform gen-ai training
        ROS will expose the basic steering, sensor and low level hw functions that the gen-ai code will use to predict series of future actions. Future actions will generate new camera data do that in turn with predict new motor actions and so on
        Think like gen-ai brain is sitting on the driver seat and based on brain prediction it will turn steering, push brakes or accelerate ( through ROS) 
        Think of collision avoidance like assisted braking in modern cars that automatically breaks using the distance estimation to the car in front.
        I was going to send this paper little late, but if you want you can read this paper. It has some context that will require some additional reading. But Justin case you want to get idea of the algorithm that we will use as a gen-ai brain using transformers. https://arxiv.org/pdf/2304.13705

Justice

        Sounds good, thank you!

