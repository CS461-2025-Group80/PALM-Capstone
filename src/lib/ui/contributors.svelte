<script lang="ts">
    import { User, ExternalLink } from 'lucide-svelte';   
    const contributors = {
        "REVOBOTS": {
            "description": "A robots company that created the PALM Capstone at Oregon State University to research and explore product feasibility.",
            "website": "https://revobots.ai/",
            "and_others": true,
            "members": [
                {
                    "name": "Rahul Khanna",
                    "description": "asdf",
                    "photo": "rahul.jpg"
                },
                {
                    "name": "Aditya Raj",
                    "description": "asdf",
                    "photo": null
                },
                {
                    "name": "Derick John",
                    "description": "asdf",
                    "photo": null
                },
                {
                    "name": "Joshua Negreanu",
                    "description": "asdf",
                    "photo": null
                }
            ]
        },
        "PALM Capstone Team": {
            "description": "Consists of Oregon State University students that volunteered to join the PALM Capstone.",
            "website": null,
            "and_others": false,
            "members": [
                {
                    "name": "Justice Peyton",
                    "description": "asdf",
                    "photo": null
                },
                {
                    "name": "Andrew Fief",
                    "description": "asdf",
                    "photo": "andrew.jpg"
                },
                {
                    "name": "Luke Hashbarger",
                    "description": "asdf",
                    "photo": "luke.jpg"
                }
            ]
        }
    } as const;
</script>

<section class="py-12 px-4 max-w-6xl mx-auto">
  <h2 class="text-4xl font-bold mb-12 text-center">Contributions</h2>
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
            <h4 class="text-lg font-semibold mb-2">{contributor.name}</h4>
            <p class="text-gray-700">{contributor.description}</p>
          </div>
        {/each}
        {#if contributors[key_casted].and_others}
        <div>
            <h4>
                ...and other members of REVOBOTS.
            </h4>
        </div>
        {/if}
      </div>
    </div>
  {/each}
</section>
