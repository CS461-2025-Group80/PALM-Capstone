<script>
    import Contributors from './contributors.svelte';
    import Header from './header.svelte';
    import Footer from './footer.svelte';
</script>

<div class="flex flex-col min-h-screen">
    <Header />
  
    <main class="flex-1 max-w-6xl mx-auto w-full px-4 py-12">
        <section class="mb-16">
            <h1 class="mb-6">PALM Capstone</h1>
            <p class="text-lg text-gray-600 mb-4">
                dud
            </p>
        </section>

        <section class="mb-16">
            <Contributors />
        </section>
    </main>
    
    <Footer />
</div>
