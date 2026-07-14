function openTab(tabId, elmnt) {
    // Hide all tabcontent
    const tabcontents = document.getElementsByClassName("tabcontent");
    for (let i = 0; i < tabcontents.length; i++) {
        tabcontents[i].classList.remove("active");
    }

    // Remove active class from buttons
    const tablinks = document.getElementsByClassName("tablink");
    for (let i = 0; i < tablinks.length; i++) {
        tablinks[i].classList.remove("active");
    }

    // Show the specific tab
    document.getElementById(tabId).classList.add("active");
    elmnt.classList.add("active");

    // Load external content
    loadTemplate(tabId);
}

function loadTemplate(tabId) {
    // Dashboard already has inline content, nothing to fetch
    // All other templates are fetched here
    if (tabId === "tables") {
        fetch("/tables")
            .then(r => r.text())
            .then(html => {
                document.getElementById("tables").innerHTML = html;
            });
    } else if (tabId === "actions") {
        fetch("/actions")
            .then(r => r.text())
            .then(html => {
                document.getElementById("actions").innerHTML = html;
            });
    }
}

window.onload = function() {
  document.getElementById("defaultOpen").click();
};