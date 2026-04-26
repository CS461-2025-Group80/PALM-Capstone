<script lang="ts">
    import { User, ExternalLink } from 'lucide-svelte';
    const contributors = {
        "REVOBOTS": {
            "description": "A robots company that created the PALM Capstone at Oregon State University to research and explore product feasibility.",
            "website": "https://revobots.ai/",
            "and_others": "...and other members of REVOBOTS.",
            "members": [
                {
                    "name": "Rahul Khanna",
                    "description": "REVOBOTS lead. Provided the team resources and motivation for development.",
                    "photo": "rahul.jpg",
                    "linkedin": "https://www.linkedin.com/in/rahul-khanna-ph-d-6a02712/"
                },
                {
                    "name": "Aditya Raj",
                    "description": "Project lead. Worked on image segmentation and developing a method of autonomous driving.",
                    "photo": "aditya.jpg",
                    "linkedin": "https://www.linkedin.com/in/aditya-raj-aadi0032007/"
                },
                {
                    "name": "Derick John",
                    "description": "Vision-focused. Worked on image segmentation for informing a robot of where to drive.",
                    "photo": "derick.jpg",
                    "linkedin": "https://www.linkedin.com/in/derick-john-67459a20a/"
                },
                {
                    "name": "Joshua Negreanu",
                    "description": "A member of a previous PALM capstone team. Provided the team resources and guidance.",
                    "photo": "joshua.jpg",
                    "linkedin": "https://www.linkedin.com/in/joshnegreanu/"
                }
            ]
        },
        "PALM Capstone Team": {
            "description": "Consists of Oregon State University students that volunteered to join the PALM Capstone.",
            "website": null,
            "and_others": null,
            "members": [
                {
                    "name": "Justice Peyton",
                    "description": "ROS- and data logging-focused. Developed LIDAR mapping and pathfinding, data logging, software reading GPS devices, and internet-based controlling and camera viewing for the MyAGV.",
                    "photo": "justice.jpg",
                    "linkedin": "https://www.linkedin.com/in/justice-peyton/"
                },
                {
                    "name": "Andrew Fief",
                    "description": "Vision-focused.",
                    "photo": "andrew.jpg",
                    "linkedin": "https://www.linkedin.com/in/andrew-fief/"
                },
                {
                    "name": "Luke Hashbarger",
                    "description": null,
                    "photo": "luke.jpg",
                    "linkedin": "https://www.linkedin.com/in/luke-hashbarger-5936a4272/"
                }
            ]
        }
    } as const;
</script>

<section class="py-12 px-4 max-w-6xl mx-auto">
  {#each Object.keys(contributors) as key}
    {@const key_casted = key as keyof (typeof contributors)}
    <div class="mb-16">
      <div class="flex items-center gap-2">
        <h3 class="text-2xl font-semibold">{key}</h3>
        {#if contributors[key_casted].website}
          <a href={contributors[key_casted].website} target="_blank" rel="noopener noreferrer" class="text-blue-500 hover:text-blue-700">
            <ExternalLink size={24} />
          </a>
        {/if}
      </div>
      <p class="text-gray-600 mb-8 mt-2">{contributors[key_casted].description}</p>
      <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
        {#each contributors[key_casted].members as contributor}
          <div class="bg-white rounded-lg shadow-md p-6 hover:shadow-lg transition-shadow">
            {#if contributor.photo}
              <img src={`/contributor_pictures/${contributor.photo}`} alt="Picture of {contributor.name}" class="w-40 h-40 object-cover rounded-md mb-4"/>
            {:else}
              <div class="w-40 h-40 bg-gray-200 rounded-md mb-4 flex items-center justify-center">
                <User size={64} class="text-gray-400" />
              </div>
            {/if}
            <div class="flex items-center gap-2 mb-2">
              <h4 class="text-lg font-semibold">{contributor.name}</h4>
              {#if contributor.linkedin}
                <a href={contributor.linkedin} target="_blank" rel="noopener noreferrer" class="text-blue-600 hover:text-blue-800">
                  <img src="/linkedin.svg" alt="LinkedIn" class="w-5 h-5" />
                </a>
              {/if}
            </div>
            {#if contributor.description}
              <p class="text-gray-700">{contributor.description}</p>
            {/if}
          </div>
        {/each}
        {#if contributors[key_casted].and_others}
        <div class="bg-white rounded-lg shadow-md p-6 hover:shadow-lg transition-shadow flex items-center justify-center min-h-full">
          <h4 class="text-lg font-semibold text-center text-gray-700">
            {contributors[key_casted].and_others}
          </h4>
        </div>
        {/if}
      </div>
    </div>
  {/each}
</section>