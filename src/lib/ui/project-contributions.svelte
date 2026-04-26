<script lang="ts">
    import { DESCRIPTION_PARAGRAPH, FIGURE_STYLES, HEADING_STYLES } from '../styles';
</script>

<div>
    <h2 class={HEADING_STYLES.h2}>LIDAR Mapping and Pathfinding</h2>
    <p class={DESCRIPTION_PARAGRAPH}>
        "LIDAR" is a method of measuring distance by bouncing light off of nearby objects and waiting to see how long it takes for the light to reflect.
        The team implemented a LIDAR-based mapping system with open-souce code.
    </p>

    <p class={DESCRIPTION_PARAGRAPH}>
        With the mapping we created with LIDAR, we, again, implemented open-source code to parse the map, derive where the robot is based on current the current surroundings of the robot (derived from LIDAR), which we could then use to
        specify where we want the robot to drive to.
    </p>

    <div class={FIGURE_STYLES.grid}>
        <figure class={FIGURE_STYLES.figure}>
            <img src="/contributions_pictures/mapping.jpg" alt="" class={FIGURE_STYLES.image}/>
            <figcaption class={FIGURE_STYLES.caption}>A mapping of Kelley Engineering Center's 3rd floor done via the MyAGV's LIDAR sensor.</figcaption>
        </figure>

        <figure class={FIGURE_STYLES.figure}>
            <img src="/contributions_pictures/localizer.png" alt="" class={FIGURE_STYLES.image}/>
            <figcaption class={FIGURE_STYLES.caption}>The MyAGV localizing itself on Kelley Engineering Center's 3rd Floor.</figcaption>
        </figure>
    </div>

    <h3 class={HEADING_STYLES.h3}>Conclusion</h3>
    <p class={DESCRIPTION_PARAGRAPH}>
        The conclusion that was reached was that mapping was too tedious to do and isn't reactionary-- REVOBOTS wants something that can work in most, if not all, environments
        automatically.
    </p>
</div>

<div>
    <h2 class={HEADING_STYLES.h2}>GPS</h2>
    <p class={DESCRIPTION_PARAGRAPH}>
        The team implemented a GPS onto the MyAGV. This required hardware for mounting the GPS and software to actually read the GPS.
    </p>
    <h3 class={HEADING_STYLES.h3}>Conclusion</h3>
    <p class={DESCRIPTION_PARAGRAPH}>
        REVOBOTS wanted us to improve our data logging by including latitude and longitude-- not just velocity data and camera frames. This also benefits future PALM
        teams that may want to use GPS.
    </p>
</div>

<div>
    <h2 class={HEADING_STYLES.h2}>Data Logging</h2>
    <p class={DESCRIPTION_PARAGRAPH}>
        Data logging is an absolute necessity for training AI. A team member focused on developing this and logged various bits of information into a JSON, including:
        the time elapsed from the start of logging, robot latitude, longitude, angular and linear velocity, the current frame visible on the robot's camera.
    </p>
    <p class={DESCRIPTION_PARAGRAPH}>
        The method of storing frames was changed a few times to minimize space. Initially, every frame was saved as a JPG on disk.
        Instead of doing that, since that's very memory-intense, we appended each frame to an MP4 file and logged the frame index in the JSON.
        MP4s naturally carry data across each frame to save data-- so this saved us a significant amount of space.
    </p>
    <h3 class={HEADING_STYLES.h3}>Conclusion</h3>
    <p class={DESCRIPTION_PARAGRAPH}>
        REVOBOTS favored the MP4 method of logging quite significantly (due to its memory efficiency) and is using this for their final product.
    </p>
</div>

<div>
    <h2 class={HEADING_STYLES.h2}>YOLO</h2>
    <p class={DESCRIPTION_PARAGRAPH}>
        yolo (andrew)
    </p>
</div>