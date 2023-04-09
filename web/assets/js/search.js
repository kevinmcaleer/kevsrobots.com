document.getElementById("search-input").addEventListener("input", async () => {
  const searchTerm = document.getElementById("search-input").value;
  const searchResults = document.getElementById("search-results");

  // Clear previous search results
  searchResults.innerHTML = "";

  if (searchTerm) {
    // Fetch and parse the YAML file
    const response = await fetch("/assets/data/data.yml");
    const yamlData = await response.text();
    const data = jsyaml.load(yamlData);

    const results = search(data, searchTerm);

    // Display search results
    for (const result of results) {
      const listItem = document.createElement("li");
      listItem.innerHTML = `<a href="${result.link}">${result.title}</a>`;
      searchResults.appendChild(listItem);
    }
  }
});

function search(data, searchTerm) {
  const results = [];

  for (const item of data) {
    const titleMatch = item.title.toLowerCase().includes(searchTerm.toLowerCase());
    const keywordMatch = item.keywords.some(keyword => keyword.toLowerCase().includes(searchTerm.toLowerCase()));

    if (titleMatch || keywordMatch) {
      results.push(item);
    }
  }

  return results;
}
