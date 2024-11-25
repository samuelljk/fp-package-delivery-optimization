const form = document.getElementById("graph-form");
const output = document.getElementById("output");

form.addEventListener("submit", async (e) => {
  e.preventDefault();
  console.log("Form submitted"); // Debugging log

  try {
    const graph = JSON.parse(document.getElementById("graph").value);
    console.log("Graph input parsed:", graph); // Debugging log

    const algorithm = document.getElementById("algorithm").value;
    const start = document.getElementById("start").value;
    const end = document.getElementById("end").value;

    console.log("Algorithm selected:", algorithm);
    console.log("Start node:", start, "End node:", end);

    let endpoint = `http://127.0.0.1:5000/${algorithm}`;
    let requestData = { graph, start };
    if (algorithm === "shortest_path") requestData.end = end;

    console.log("Request data:", requestData);

    const response = await fetch(endpoint, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(requestData),
    });

    console.log("Response received:", response);

    if (!response.ok) {
      throw new Error(`HTTP error! Status: ${response.status}`);
    }

    const result = await response.json();
    console.log("Result from backend:", result);

    output.textContent = JSON.stringify(result, null, 2);
  } catch (error) {
    console.error("Error occurred:", error);
    output.textContent = `Error: ${error.message}`;
  }
});
