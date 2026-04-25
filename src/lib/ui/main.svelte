<script>
    import ProjectContributors from './project-contributors.svelte';
    import ProjectDescription from './project-description.svelte';
    import ProjectContributions from './project-contributions.svelte';
    import Header from './header.svelte';
    import Footer from './footer.svelte';
</script>

<div class="flex flex-col min-h-screen">
    <Header />
  
    <main class="flex-1 max-w-6xl mx-auto w-full px-4 py-12">
        <h1 class="text-4xl font-bold mb-12 text-center">PALM</h1>
        <section class="mb-16">
            <ProjectDescription />
        </section>

        <h1 class="text-4xl font-bold mb-12 text-center">PALM Contributions</h1>
        <section class="mb-16">
            <ProjectContributions />
        </section>

        <h1 class="text-4xl font-bold mb-12 text-center">Contributors</h1>
        <section class="mb-16">
            <ProjectContributors />
        </section>
    </main>
    
    <Footer />
</div>
