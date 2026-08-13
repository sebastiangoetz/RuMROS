(function () {
    // Optional per-page override: define window.settingsCogConfig BEFORE loading this script.
    //   target: CSS selector of the element to mirror (default: "#table-container")
    //   corner: "top-left" | "top-right" | "bottom-left" | "bottom-right" (default: "bottom-right")
    //   margin: inset from the chosen corner in px (default: 20)
    const DEFAULT_CONFIG = {
        target: "#table-container",
        corner: "bottom-right",
        margin: 20
    };

    function resolveConfig() {
        return Object.assign({}, DEFAULT_CONFIG, window.settingsCogConfig || {});
    }

    let overlay, cog, popup, targetSelector, corner, margin;

    // Keep the invisible overlay exactly on top of the target element
    function syncOverlay() {
        const target = document.querySelector(targetSelector);
        if (!target || !overlay) return;

        const rect = target.getBoundingClientRect();
        // Position: fixed => rect coords (viewport-relative) map 1:1
        overlay.style.top    = rect.top + "px";
        overlay.style.left   = rect.left + "px";
        overlay.style.width  = rect.width + "px";
        overlay.style.height = rect.height + "px";
    }

    function positionCog() {
        const [vSide, hSide] = corner.split("-"); // e.g. ["bottom","right"]
        const gap = 58; // space between cog and popup

        ["top", "right", "bottom", "left"].forEach(s => {
            cog.style[s] = "";
            popup.style[s] = "";
        });

        cog.style[hSide] = margin + "px";
        popup.style[hSide] = margin + "px";

        cog.style[vSide] = margin + "px";
        if (vSide === "bottom") {
            popup.style.bottom = (margin + gap) + "px";
        } else {
            popup.style.top = (margin + gap) + "px";
        }
    }

    function buildSettingsUI() {
        const cfg = resolveConfig();
        targetSelector = cfg.target;
        corner = cfg.corner;
        margin = cfg.margin;

        // --- Invisible, click-through overlay (the "ghost copy" of table-container) ---
        overlay = document.createElement("div");
        overlay.id = "settings-overlay";

        // --- Cog button ---
        cog = document.createElement("button");
        cog.id = "settings-cog";
        cog.title = "Table settings";
        cog.innerHTML = '<i class="fa-solid fa-gear"></i>';

        // --- Popup ---
        popup = document.createElement("div");
        popup.id = "settings-popup";
        popup.classList.add("hidden");
        popup.innerHTML = `
            <div class="settings-header">
                <span>Hidden tables</span>
                <button id="settings-close" title="Close">
                    <i class="fa-solid fa-xmark"></i>
                </button>
            </div>
            <p class="settings-hint">
                Add a top-level object name (e.g. "CommandManager") to hide its
                table. Changes apply automatically.
            </p>
            <ul id="settings-hidden-list"></ul>
            <div class="settings-add-row">
                <input id="settings-add-input" type="text"
                       placeholder="Object name to hide…" />
                <button id="settings-add-btn" title="Add">
                    <i class="fa-solid fa-plus"></i>
                </button>
            </div>
        `;

        overlay.appendChild(cog);
        overlay.appendChild(popup);
        document.body.appendChild(overlay); // stable parent: never cleared by processData

        positionCog();
        syncOverlay();

        // ---- list rendering + events ----
        const listEl = popup.querySelector("#settings-hidden-list");
        const inputEl = popup.querySelector("#settings-add-input");

        function renderList() {
            listEl.innerHTML = "";
            const items = getUserHiddenObjects();

            if (items.length === 0) {
                const empty = document.createElement("li");
                empty.className = "settings-empty";
                empty.textContent = "No custom hidden objects.";
                listEl.appendChild(empty);
            }

            items.forEach((name, index) => {
                const li = document.createElement("li");
                const label = document.createElement("span");
                label.textContent = name;

                const remove = document.createElement("button");
                remove.title = "Remove";
                remove.innerHTML = '<i class="fa-solid fa-trash"></i>';
                remove.addEventListener("click", () => {
                    const updated = getUserHiddenObjects();
                    updated.splice(index, 1);
                    setUserHiddenObjects(updated);
                    renderList();
                });

                li.appendChild(label);
                li.appendChild(remove);
                listEl.appendChild(li);
            });

            DEFAULT_HIDDEN_OBJECTS.forEach(name => {
                const li = document.createElement("li");
                li.className = "settings-default";
                li.textContent = name + " (default)";
                listEl.appendChild(li);
            });
        }

        function addCurrentInput() {
            const value = inputEl.value.trim();
            if (!value) return;
            const items = getUserHiddenObjects();
            if (!items.includes(value)) {
                items.push(value);
                setUserHiddenObjects(items);
            }
            inputEl.value = "";
            renderList();
        }

        cog.addEventListener("click", () => {
            popup.classList.toggle("hidden");
            if (!popup.classList.contains("hidden")) {
                renderList();
                inputEl.focus();
            }
        });
        popup.querySelector("#settings-close")
             .addEventListener("click", () => popup.classList.add("hidden"));
        popup.querySelector("#settings-add-btn")
             .addEventListener("click", addCurrentInput);
        inputEl.addEventListener("keydown", (e) => {
            if (e.key === "Enter") addCurrentInput();
        });

        renderList();

        // ---- keep overlay aligned to the (re-rendering) target ----
        window.addEventListener("resize", syncOverlay);
        window.addEventListener("scroll", syncOverlay, true); // capture: catch scrolls in inner containers too
        // table-container is rebuilt every second -> its size may change -> re-sync
        document.addEventListener("contentUpdated", syncOverlay);

        // Extra safety: observe the target's size directly
        const target = document.querySelector(targetSelector);
        if (target && "ResizeObserver" in window) {
            new ResizeObserver(syncOverlay).observe(target);
        }
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", buildSettingsUI);
    } else {
        buildSettingsUI();
    }
})();
