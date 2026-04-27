<script lang="ts">
    import { LARGE_DESCRIPTION, FIGURE_STYLES, HEADING_STYLES } from '../styles';
</script>

<div>
    <h2 class={HEADING_STYLES.h2}>Project Overview</h2>
    <p class={LARGE_DESCRIPTION}>
        The PALM project was created by the startup REVOBOTS for exploring and researching feasible ideas for the Taskbot Scout.
        The Taskbot Scout is an autonomous robot that drives in parking lots to scan for parking violations and report any to campus parking enforcement.
        The PALM project works in parallel with this product's development.

        You can view our github repository here: 
        <a href="https://github.com/CS461-2025-Group80/PALM-Capstone" target="_blank" rel="noopener noreferrer">https://github.com/CS461-2025-Group80/PALM-Capstone </a>
    </p>
</div>

<div>
    <h2 class={HEADING_STYLES.h2}>ROS</h2>
    <p class={LARGE_DESCRIPTION}>
        The company and the team use Robot Operating System 2 (ROS2) to manage all hardware and software of the robot.
        This keeps things very high-level, as ROS2 has a massive community, with years of development spent on its API.
        On top of the high-level nature, ROS2 is made very portably– so code you make on one robot is very likely to work on another,
        given that the hardware is available.
    </p>
</div>

<div>
    <h2 class={HEADING_STYLES.h2}>Project Robots</h2>
    <p class={LARGE_DESCRIPTION}>
        The team worked on two robots throughout the whole project: a MyAGV and a Turtlebot 3 Waffle.
        The Turtlebot and the MyAGV both come with open-source ROS2 code. This allowed us to immediately start working on the project.
    </p>

    <div>
        <div class={FIGURE_STYLES.grid}>
            <figure class={FIGURE_STYLES.figure}>
                <img src="/contributions_pictures/turtlebot.jpg" alt="A Turtlebot 3 Waffle. A small, square-sized robot with 2 wheels in the front. Multi-layered for modular attachment of hardware." class={FIGURE_STYLES.image_robot}/>
                <figcaption class={FIGURE_STYLES.caption}>The Turtlebot 3 Waffle.</figcaption>
            </figure>

            <figure class={FIGURE_STYLES.figure}>
                <img src="/contributions_pictures/agv.png" alt="A MyAGV from Elephant Robotics, a grey 4-wheeled shoebox-sized robot." class={FIGURE_STYLES.image_robot}/>
                <figcaption class={FIGURE_STYLES.caption}>The MyAGV from Elephant Robotics.</figcaption>
            </figure>
        </div>
    </div>
</div>

<div>
    <h2 class={HEADING_STYLES.h2}>PALM Contributions & Experiments</h2>
    <p class={LARGE_DESCRIPTION}>
        The Capstone team develops on a MyAGV, an Elephant Robotics-branded robot, while REVOBOTS works with the final product, the Taskbot Scout.
        The Capstone team has focused on a variety of topics: ROS hardware implementation, LiDAR mapping, map traversal, Teleoperation, GPS implementation, 
        video and GPS logging, YOLO, and hardware setup for future REVOBOTS Capstone teams.
        <br>
        The purpose of PALM adapted into becoming about feasibility research and experimenting with various strategies. This is elaborated more on below.
    </p>
</div>
