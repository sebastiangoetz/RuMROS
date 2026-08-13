// behavior.js
// Render BehaviorModel into DOM containers

// Tooltip helpers
let tooltipNodeId = null;   // ID of node whose tooltip is currently shown
let tooltipEl = null;

function buildTooltipContent(node, data) {
    const action = node.data("action") || {};
    const parameters = node.data("parameters") || {};
    const title = action.label || (node.data("label") || "").split("\n")[0];

    let rows = "";
    parameters.forEach(p => {
        let name = p.children.name;
        let value = p.children.value;

        // Treat certain parameters specially for better display
        if (p.type.includes("AreaParameter")) {
            value = data.Area[value].children.name;
        }

        // Denote NaN numeric parameters as dynamic
        if ((p.type.includes("DoubleParameter") || p.type.includes("IntParameter")) && isNaN(value)) {
            value = "<i>dynamically assigned</i>";
        }

        rows += `<div><span style="font-weight:bold">${name}:</span> ${value}</div>`;
    });
    if (!rows) rows = "<div><i>No parameters</i></div>";

    return `<div style="font-weight:bold;margin-bottom:4px;color:var(--col-accent)">${title}</div>${rows}`;
}

function positionTooltip(node) {
    if (!tooltipEl || node.empty()) return;
    const pos = node.renderedPosition();
    const h = node.height() * cy.zoom();
    tooltipEl.style.left = pos.x + "px";
    tooltipEl.style.top = (pos.y - h / 2 - 10) + "px";
}

function showTooltip(node, data) {
    tooltipNodeId = node.id();
    if (!tooltipEl) {
        tooltipEl = document.createElement("div");
        Object.assign(tooltipEl.style, {
            position: "absolute",
            zIndex: "9999",
            background: "rgba(255,255,255,0.98)",
            border: "1px solid var(--col-accent)",
            borderRadius: "6px",
            padding: "8px 12px",
            fontSize: "13px",
            maxWidth: "260px",
            pointerEvents: "none",
            boxShadow: "0 2px 8px rgba(0,0,0,0.2)",
            transform: "translate(-50%, -100%)"
        });
        cy.container().appendChild(tooltipEl);
    }
    refreshTooltip(data);
}

function refreshTooltip(data) {
    if (!tooltipNodeId || !tooltipEl) return;
    const node = cy.getElementById(tooltipNodeId);
    // Node may have been removed, or is no longer a leaf BB -> hide
    if (node.empty() || node.data("type") !== "BB") {
        hideTooltip();
        return;
    }
    tooltipEl.innerHTML = buildTooltipContent(node, data);
    tooltipEl.style.display = "block";
    positionTooltip(node);
}

function hideTooltip() {
    tooltipNodeId = null;
    if (tooltipEl) tooltipEl.style.display = "none";
}


// Badge for behaviors that are start behaviors within CBs (all direct children in a striped sequence CB or first in a normal CB)
const startMarker =
  'data:image/svg+xml;utf8,' +
  encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16"><circle cx="8" cy="8" r="6" fill="rgba(192, 28, 40, 1.00)"/></svg>');

const layoutParamsFcose = {
    name: 'fcose',
    quality: 'proof',
    randomize: true,
    animate: false,
    fit: true,
    padding: 40,
    nodeDimensionsIncludeLabels: true,
    uniformNodeDimensions: false,

    // Force-based separation
    nodeSeparation: 100,
    nodeRepulsion: 6000,
    nestingFactor: 0.1,
    gravity: 0.5,
    gravityRange: 3.8,
    gravityCompound: 0.5,
    gravityRangeCompound: 1.5,

    // Connected node distances
    idealEdgeLength: 120,
    edgeElasticity: 0.8,
    numIter: 50,
    initialEnergyOnIncremental: 0.3
};

function createSmallHeading(heading, container) {
    var text = document.createElement("span");
    text.style.fontWeight = "bold";
    text.style.marginBottom = "5px";
    text.appendChild(document.createTextNode(heading));
    container.appendChild(text);
}

function createHeading(heading, container) {
    var h1 = document.createElement("h1");
    h1.style.margin = "20px";
    h1.style.color = "var(--col-accent)"
    h1.appendChild(document.createTextNode(heading));
    container.appendChild(h1);

    const sep = document.createElement("hr");
    sep.style.border = "1px solid #ccc";
    sep.style.margin = "20px 0";
    container.appendChild(sep);
}

function getObjectById(arr, id) {
    return (arr.filter(i => i.id === id) || [null])[0];
}

function getFirstBB(behavior) {
    const type = getConcreteType(behavior.type);
    if (type.includes("BB")) return behavior.id;
    if (type.includes("CB")) {
        const subs = behavior.children?.Behavior || [];
        for (let sb of subs) {
            const bbId = getFirstBB(sb);
            if (bbId) return bbId;
        }
    }
    return null;
}

// https://stackoverflow.com/a/1584377
const merge = (a, b, predicate = (a, b) => a === b) => {
    const c = [...a]; // copy to avoid side effects
    // add all items from B to copy C if they're not already present
    b.forEach((bItem) => (c.some((cItem) => predicate(bItem, cItem)) ? null : c.push(bItem)))
    return c;
}

// Cytoscape does not support striped gradients, so a custom image is used
function makeStripedSVG(color1, color2, angle = 45, stripeCount = 10, stripeWidth = 4, stripeSpacing = 4) {
    const totalWidth = stripeCount * (stripeWidth + stripeSpacing);
    let rects = '';

    for (let i = 0; i < stripeCount; i++) {
        const x = i * (stripeWidth + stripeSpacing);
        rects += `<rect x="${x}" width="${stripeWidth}" height="${totalWidth * 2}" fill="${color1}" />`;
    }

    const svg = `
        <svg xmlns="http://www.w3.org/2000/svg" width="${totalWidth}" height="${totalWidth}">
        <defs>
            <pattern id="stripes" patternUnits="userSpaceOnUse"
                width="${totalWidth}" height="${totalWidth}"
                patternTransform="rotate(${angle})">
            <rect width="100%" height="100%" fill="${color2}" />
            ${rects}
            </pattern>
        </defs>
        <rect width="100%" height="100%" fill="url(#stripes)" />
        </svg>
    `;
    return `data:image/svg+xml;base64,${btoa(svg)}`;
}

// Create stripe gradient BGs
const stripedInactive = makeStripedSVG(colInactive, colInactiveLight, 45, 10, 3, 3);
const stripedInactiveL = makeStripedSVG(colInactive, colInactiveLight, 45, 3, 3, 3);
const stripedActive = makeStripedSVG(colAccentLight, colAccentLightMuted, 45, 10, 3, 3);
const stripedActiveL = makeStripedSVG(colAccentLight, colAccentLightMuted, 45, 3, 3, 3);
const stripedStart = makeStripedSVG(colEmphasizedMid, colEmphasizedLight, 45, 10, 3, 3);
const stripedStartL = makeStripedSVG(colEmphasizedMid, colEmphasizedLight, 45, 3, 3, 3);
const legendStates = [stripedInactiveL, stripedActiveL, stripedStartL];
const legendOutlines = [colInactive, colAccent, colEmphasized];

function createSettings(behaviorModel, container) {
    const settingsContainer = document.createElement("div");
    
    createHeading("Settings and Actions", settingsContainer);

    var viewP = document.createElement("p");
    viewP.style.fontWeight = "bold";
    viewP.innerHTML = "View";
    settingsContainer.appendChild(viewP);

    const autoLayoutButton = document.createElement("button");
    autoLayoutButton.textContent = "Auto-arrange nodes (Fcose)";
    autoLayoutButton.style.padding = "10px 20px";
    autoLayoutButton.style.fontSize = "16px";

    autoLayoutButton.addEventListener("click", (e) => {
        runLayoutReliably(layoutParamsFcose);
    });

    settingsContainer.appendChild(autoLayoutButton);


    var statusP = document.createElement("p");
    statusP.style.fontWeight = "bold";
    statusP.innerHTML = "Status";
    settingsContainer.appendChild(statusP);

    const statusDiv = document.createElement("div");
    statusDiv.style.margin = "10px";
    statusDiv.style.padding = "10px";
    statusDiv.style.border = "1px solid #ddd";
    statusDiv.style.backgroundColor = "#f9f9f9";

    // Show current time if > 0
    let currentTime = behaviorModel.children.currentTime;
    if (currentTime > 0) {
        const timeP = document.createElement("p");
        const date = new Date(currentTime);
        timeP.textContent = "Current Time: " + date.toLocaleString();
        statusDiv.appendChild(timeP);
    }

    // Show running state
    const runningP = document.createElement("p");
    runningP.textContent = "Running: " + (behaviorModel.children.running ? "Yes" : "No");
    statusDiv.appendChild(runningP);

    // Show StopAction name
    const stopAction = behaviorModel.children.StopAction;
    if (stopAction && stopAction.children && stopAction.children.label) {
        const stopActionP = document.createElement("p");
        stopActionP.textContent = "Stop Action: " + stopAction.children.label;
        statusDiv.appendChild(stopActionP);
    }

    settingsContainer.appendChild(statusDiv);

    const form = document.createElement("form");
    form.style.margin = "20px 0";

    const startStopButton = document.createElement("button");
    startStopButton.type = "submit";
    startStopButton.textContent = behaviorModel.children.running ? "Stop" : "Start";
    startStopButton.style.padding = "10px 20px";
    startStopButton.style.fontSize = "16px";

    // Handle form submission
    form.addEventListener("submit", (e) => {
        e.preventDefault();
        const cmdData = { type: "toggleBehaviorModel" };
        socket.emit("sendCommand", cmdData);
    });

    form.appendChild(startStopButton);
    settingsContainer.appendChild(form);
    container.appendChild(settingsContainer);
}

let cLegend3 = 0; // Counts cycles mod 3 for background switches
let cLegendArrowColor = "var(--col-inactive)";
function createLegend(container) {
    function addLegendItem(ctn, label, color = null, colorOutline = null, striped = false, round = false) {
        if (color == null && colorOutline == null && !striped)
            throw new Error("Legend item must have either color, outline color or both");

        const item = document.createElement("div");
        item.style.display = "flex";
        item.style.alignItems = "center";
        item.style.marginBottom = "10px";
        item.style.marginLeft = "10px";

        const box = document.createElement("span");
        box.style.display = "inline-block";
        box.style.width = "14px";
        box.style.height = "14px";
        box.style.marginRight = "8px";
        box.style.borderRadius = round ? "50%" : "3px";

        if (striped) {
            
            box.style.backgroundImage = `url(${legendStates[cLegend3]})`;
            colorOutline = legendOutlines[cLegend3];

            box.style.backgroundSize = "cover";
            cLegend3 = (cLegend3 + 1) % legendStates.length;
        } else {
            if (color != null)
                box.style.backgroundColor = color;
        }

        box.style.border = "2px solid " + (colorOutline || color);

        const text = document.createElement("span");
        text.textContent = label;

        item.appendChild(box);
        item.appendChild(text);
        ctn.appendChild(item);
    }

    function addLegendArrow(ctn, label, colorA = "var(--col-inactive)", colorB = null, dashed = false) {
        const item = document.createElement("div");
        item.style.display = "flex";
        item.style.alignItems = "center";
        item.style.marginBottom = "4px";
        item.style.marginLeft = "10px";

        const arrow = document.createElement("span");
        arrow.style.display = "inline-flex";
        arrow.style.justifyContent = "center";
        arrow.style.alignItems = "center";
        arrow.style.marginRight = "8px";
        arrow.style.fontSize = "22px";
        arrow.style.fontWeight = "bold";
        arrow.innerHTML = dashed ? "&#8672;" : "&#8592;"; // Left arrow (dashed / normal)
        arrow.style.color = colorA;

        const text = document.createElement("span");
        text.textContent = label;

        item.appendChild(arrow);
        item.appendChild(text);
        ctn.appendChild(item);

        // Animate color cycling if a second color is provided
        if (colorB) {
            arrow.style.color = cLegendArrowColor;
            cLegendArrowColor = cLegendArrowColor === colorA ? colorB : colorA;
        }
    }

    const legendContainer = document.createElement("div");

    createHeading("Legend", legendContainer);

    createSmallHeading("Complex Behavior Blocks (CBs)", legendContainer);
    addLegendItem(legendContainer, "Startup CB", "var(--col-emphasized-light)", "var(--col-emphasized)");
    addLegendItem(legendContainer, "Standard CB", "var(--col-accent-light-muted)", "var(--col-accent)");
    addLegendItem(legendContainer, "Parallel CB (starts contained BBs all at once)", null, null, true);

    createSmallHeading("Basic Behaviors (BBs, contained within CBs)", legendContainer);
    addLegendItem(legendContainer, "Startup BB", "var(--col-emphasized)");
    addLegendItem(legendContainer, "Active BB", "var(--col-accent)");
    addLegendItem(legendContainer, "Inactive BB", "var(--col-inactive-light)");
    addLegendItem(legendContainer, "Starts on container CB start", "rgba(192, 28, 40, 1.00)", null, false, true);

    createSmallHeading("Transitions", legendContainer);
    addLegendArrow(legendContainer, "Inactive transition", "var(--col-inactive)"); 
    addLegendArrow(legendContainer, "Active transition", colAccent);
    addLegendArrow(legendContainer, "Conditional transition", "var(--col-inactive)", "var(--col-accent)", true);
    addLegendArrow(legendContainer, "Mutually exclusive transition(s)", colMutuallyExclusive);

    container.appendChild(legendContainer);
}

function renderBehaviorModelSettings(data, containerId) {
    const behaviorModel = getObjectsOfType(data, "BehaviorModel")[0];

    const container = document.getElementById(containerId);
    container.innerHTML = ""; // Clear previous render

    createLegend(container);
    createSettings(behaviorModel, container);
}

let cy = null;
function initBehaviorGraph(containerId, data) {
    cy = cytoscape({
        container: document.getElementById(containerId),
        layout: { name: "breadthfirst", rankDir: "LR", nodeSep: 30, rankSep: 30 },
        style: [
            // === Base ===
            {
                selector: "node",
                style: {
                    "label": "data(label)",
                    "shape": "round-rectangle",
                    "text-wrap": "wrap",
                    "font-size": 20,
                    "color": "#000",
                    "width": 220,
                    "height": "label",
                    "padding": "10px",
                    "text-max-width": "210px",
                    "text-valign": n => n.data("type") === "CB" ? "top" : "center",
                    "text-halign": "center",
                    "text-margin-y": n => n.data("type") === "CB" ? "-10px" : "0",
                    "background-color": colInactive,
                    "border-color": colInactiveDark,
                    "border-width": 3
                }
            },
            // === CB base ===
            {
                selector: 'node[type="CB"]',
                style: {
                    "background-color": colAccentLightMuted,
                    "border-color": colAccent
                }
            },

            // === BB base ===
            {
                selector: 'node[type="BB"]',
                style: {
                    "background-color": colInactiveLight,
                    "border-color": colInactiveLight
                }
            },

            // === Start nodes (shown when !isRunning) ===
            {
                selector: 'node[type="CB"].node-start',
                style: {
                    "background-color": colEmphasizedLight,
                    "border-color": colEmphasized
                }
            },
            {
                selector: 'node[type="BB"].node-start',
                style: {
                    "background-color": colEmphasized,
                    "border-color": colEmphasized
                }
            },

            // === Active BBs (only applied when isRunning && node.state.Active) ===
            {
                selector: 'node[type="BB"].node-active',
                style: {
                    "background-color": colAccent,
                    "border-color": colAccent,
                    "border-width": 4
                }
            },
            {
                selector: 'node.node-sequence',
                style: {
                    "background-image": stripedInactive,
                    "background-fit": "cover",
                    "border-color": "#888"
                }
            },
            {
                selector: 'node.node-sequence.node-on-cb-start',
                style: {
                    "background-image": [stripedInactive, startMarker],
                    "background-fit": ["cover", "none"],
                    "background-width": ["auto", "16px"],
                    "background-height": ["auto", "16px"],
                    "background-position-x": ["0%", "100%"],
                    "background-position-y": ["0%", "0%"],
                    "background-image-containment": ["inside", "over"]
                }
            },
            {
                selector: 'node.node-sequence.node-active',
                style: {
                    "background-image": stripedActive,
                    "border-color": colAccent,
                    "border-width": 4
                }
            },
            {
                selector: 'node.node-sequence.node-active.node-on-cb-start',
                style: {
                    "background-image": [stripedActive, startMarker],
                    "background-fit": ["cover", "none"],
                    "background-width": ["auto", "16px"],
                    "background-height": ["auto", "16px"],
                    "background-position-x": ["0%", "100%"],
                    "background-position-y": ["0%", "0%"],
                    "background-image-containment": ["inside", "over"]
                }
            },
            {
                selector: 'node.node-sequence.node-start',
                style: {
                    "background-image": stripedStart,
                    "border-color": colEmphasized,
                    "border-width": 4
                }
            },
            {
                selector: 'node.node-sequence.node-start.node-on-cb-start',
                style: {
                    "background-image": [stripedStart, startMarker],
                    "background-fit": ["cover", "none"],
                    "background-width": ["auto", "16px"],
                    "background-height": ["auto", "16px"],
                    "background-position-x": ["0%", "100%"],
                    "background-position-y": ["0%", "0%"],
                    "background-image-containment": ["inside", "over"]
                }
            },
            {
                selector: "node.node-on-cb-start:childless",
                style: {
                    "background-image": startMarker,
                    "background-width": "16px",
                    "background-height": "16px",
                    "background-position-x": "100%",
                    "background-position-y": "0%",
                    "background-clip": "none",
                    "background-image-containment": "over",
                    "background-fit": "none"
                }
            },

            // === Edges ===
            {
                selector: 'edge:loop',
                style: {
                    "loop-direction": "90deg", 
                    "loop-sweep": "-20deg",
                    "target-endpoint": "90deg",
                    "source-endpoint": "105deg",
                    "control-point-step-size": 120
                }
            },
            {
                selector: "edge",
                style: {
                    "width": 6,
                    "line-color": colInactive,
                    "target-arrow-color": colInactive,
                    "target-arrow-shape": "triangle",
                    "curve-style": "bezier",
                    "label": "data(label)",
                    "font-size": 12
                }
            },
            {
                selector: "edge.edge-from-active",
                style: {
                    "line-color": colAccent,
                    "target-arrow-color": colAccent
                }
            },
            {
                selector: 'edge[type="TConditional"]',
                style: {
                    "line-style": "dashed"
                }
            },
            {
                selector: 'edge[isMutuallyExclusive = "true"]',
                style: {
                    "line-color": colMutuallyExclusive,
                    "target-arrow-color": colMutuallyExclusive
                }
            }
        ]
    });

    // Ensure overlay tooltip positions relative to the graph container
    cy.container().style.position = "relative";

    // Show tooltip only for leaf BB nodes (not composite CB nodes)
    cy.on("tap", "node", (evt) => {
        const node = evt.target;
        if (node.data("type") !== "BB" || !node.isChildless()) return;
        showTooltip(node, data);
    });

    // Click on empty canvas dismisses the tooltip
    cy.on("tap", (evt) => {
        if (evt.target === cy) hideTooltip();
    });

    // Keep tooltip glued to the node while panning/zooming
    cy.on("pan zoom", () => {
        if (tooltipNodeId) positionTooltip(cy.getElementById(tooltipNodeId));
    });
}

function updateBehaviorGraph(data) {
    if (!cy) return;

    const { cyNodes, cyEdges, startBehaviors, isRunning } = buildBehaviorElements(data);

    // Update or add nodes
    cyNodes.forEach(n => {
        const node = cy.getElementById(n.data.id);
        if (node.nonempty()) {
            // Update data only, preserving position & manual layout
            node.data(n.data);
        } else {
            cy.add(n);
        }
    });

    // Remove nodes that no longer exist
    cy.nodes().forEach(node => {
        if (!cyNodes.some(n => n.data.id === node.id())) {
            cy.remove(node);
        }
    });

    // Update or add edges
    cyEdges.forEach(e => {
        const edge = cy.getElementById(e.data.id);
        if (edge.nonempty()) {
            edge.data(e.data);
        } else {
            cy.add(e);
        }
    });

    // Remove edges that no longer exist
    cy.edges().forEach(edge => {
        if (!cyEdges.some(e => e.data.id === edge.id())) {
            cy.remove(edge);
        }
    });

    // Update states & classes
    cy.nodes().forEach(node => {
        const n = cyNodes.find(n => n.data.id === node.id());
        if (!n) return;

        node.removeClass("node-active node-start node-sequence node-on-cb-start");

        const state = n.data.state || {};
        const isStart = startBehaviors.includes(n.data.id);

        if (!isRunning) {
            if (isStart)
                node.addClass("node-start");
        } else {
            if (state.Active)
                node.addClass("node-active");
        }

        if (state.Sequence)
            node.addClass("node-sequence");

        if (n.data.isStartBehavior)
            node.addClass("node-on-cb-start");
    });

    cy.edges().forEach(edge => {
        const src = cy.getElementById(edge.data("source"));
        if (src.hasClass("node-active"))
            edge.addClass("edge-from-active");
        else
            edge.removeClass("edge-from-active");
    });

    // Re-render tooltip content/position so it survives periodic updates
    refreshTooltip(data);
}

async function runLayoutReliably(opts) {
    // Wait for fonts.
    if (document.fonts && document.fonts.ready) {
        await document.fonts.ready;
    }

    // Ensure the container has a real, non-zero size, and that
    // Cytoscape knows about it
    const c = cy.container();
    if (!c || c.offsetWidth === 0 || c.offsetHeight === 0) {
        // Container not visible yet
        return false;
    }
    cy.resize();   // Re-sync Cytoscape to current size

    // Force label/dimension computation synchronously
    // Reading boundingBox() triggers measurement pass
    cy.nodes().forEach(n => n.boundingBox());

    const layout = cy.layout(opts);
    const done = layout.promiseOn('layoutstop');
    layout.run();
    await done;
    return true;
}

// Render with Cytoscape
let firstRun = true;
function renderBehaviorModel(data, containerId) {
    if (!cy) {
        initBehaviorGraph(containerId, data);
    }
    updateBehaviorGraph(data);

    if (firstRun) {
        if (typeof cytoscapeFcose === 'undefined') {
            console.warn("cytoscape-fcose could not be loaded, using breadthfirst layout instead");
            runLayoutReliably({ name: 'breadthfirst', rankDir: 'LR' });
        }
        else {
            cytoscape.use(cytoscapeFcose);
            runLayoutReliably(layoutParamsFcose);
        }

        firstRun = false;
    }
}

function buildBehaviorElements(data) {
    // Helper to get action ID from behavior
    function getActionId(b) {
        const { relations } = b;

        if (!relations) return undefined;

        if (Array.isArray(relations)) {
            return relations.find(r => "Action" in r)?.Action;
        }

        return relations.Action;
    }

    const bm = data.BehaviorModel;
    if (!bm) throw new Error("Could not find BehaviorModel in data");

    // When launched with Gazebo, there may not be groups
    let groups = [];
    if (data.SwarmGroup)
        groups = data.SwarmGroup;

    const units = data.SwarmUnit;
    if (!units) throw new Error("Could not find SwarmUnits in data");

    if (!data.Action) throw new Error("Could not find Actions in data");
    const actions = new Map(data.Action.map(a => [a.id, a]));

    let startBehaviors = bm.relations?.StartBehavior;
    const isRunning = bm.children?.running;

    // Flatten behaviors into a map with parent relationships
    function collectBehaviors(cb, parentChain = []) {
        const nodes = [];
        const cbId = cb.id;
        const cbChildren = cb.children || {};
        const newParentChain = parentChain.concat(cbId);

        // Add the CB node itself as an abstract node
        nodes.push({
            id: cbId,
            type: "CB",
            label: cbChildren.name || `CB ${cbId}`,
            children: cb.children?.Behavior || [],
            state: cbChildren,
            transitionsMutuallyExclusive: cbChildren?.TransitionsMutuallyExclusive || false,
            parentChain: parentChain
        });

        const isSequence = !!cbChildren.Sequence;
        const behaviors = cb.children?.Behavior || [];

        // Recurse
        behaviors.forEach((b, index) => {
            const isStart = isSequence || index === 0;
            const type = getConcreteType(b.type);
            if (type.includes("BB")) {
                // Get action
                let a = actions.get(getActionId(b));
                nodes.push({
                    id: b.id,
                    type: "BB",
                    label: (a ? ("BB " + (b.children?.id ?? "") + ": " + "\n" + a.children.label) : ("BB " + (b.children?.id ?? b.id))),
                    state: b.children,
                    action: a ? a.children : null,
                    parameters: b.children?.Parameter ?? [],
                    parentChain: newParentChain,
                    transitionsMutuallyExclusive: b.children?.TransitionsMutuallyExclusive || false,
                    isStartBehavior: isStart
                });
            } else if (type.includes("CB")) {
                const childNodes = collectBehaviors(b, newParentChain);
                // Mark the first child as a start behavior if this CB is a sequence or the first in its parent
                if (childNodes.length) childNodes[0].isStartBehavior = isStart;
                nodes.push(...childNodes);
            }
        });

        return nodes;
    }

    // Create node/edge data
    const behaviorNodes = [];
    (bm.children?.CB || []).forEach(cb => {
        behaviorNodes.push(...collectBehaviors(cb, []));
    });

    // Collect all startup behavior IDs
    let startupBBs = [];
    (startBehaviors || []).forEach(id => {
        const cb = getObjectById(behaviorNodes, id);
        if (!cb) return;
        const behaviors = cb.children;
        if (!behaviors || behaviors.length == 0) return;

        if (cb.state.Sequence) {
            behaviors.forEach(b => {
                const bbId = getFirstBB(b);
                if (bbId) startupBBs.push(bbId);
            });
        } else {
            const bbId = getFirstBB(behaviors[0]);
            if (bbId) startupBBs.push(bbId);
        }
    });

    // Deduplicate to be extra safe
    startBehaviors = merge(startBehaviors, startupBBs);

    // Map by id for quick access
    const nodesById = {};
    behaviorNodes.forEach(n => nodesById[n.id] = n);

    // Edges from Transition relations
    const edges = [];
    // Helper: find transitions on a behavior node (CB or BB)
    function collectTransitions(nodeObj) {
        // Helper: find behavior by ID
        function findBehaviorObj(id, list) {
            for (const item of list) {
                if (item.id == id) return item;
                if (item.children && item.children?.Behavior) {
                    const found = findBehaviorObj(id, item.children.Behavior);
                    if (found) return found;
                }
            }
            return null;
        }
        return findBehaviorObj(nodeObj.id, bm.children?.CB || []);
    }

    // Iterate behaviors and collect transitions
    behaviorNodes.forEach(n => {
        const raw = collectTransitions(n);
        if (!raw) return;
        const trans = raw.children?.Transition || raw.Transition || [];
        (trans || []).forEach(t => {
            const target = t.relations?.NextBehavior;
            if (!target) return;
            // Only add edge if both nodes exist in our flattened set
            if (nodesById[n.id] && nodesById[target]) {
                edges.push({
                    id: `${n.id}->${target}::${t.id}`,
                    source: n.id,
                    target: target,
                    mutuallyExclusive: n.transitionsMutuallyExclusive,
                    raw: t
                });
            }
        });
    });

    // Include nested parent and additional info in nodes
    const cyNodes = behaviorNodes.map(n => {
    let label = n.label;
    if (n.type === 'BB') {
        const st = n.state || {};
        const infoLines = [
        st.LastStart > 0 ? `Last start: ${new Date(st.LastStart).toLocaleTimeString()}` : "Last start: Never",
        st.LastEnd > 0 ? `Last stop: ${new Date(st.LastEnd).toLocaleTimeString()}` : "Last stop: Never",
        st.TargetGroupId >= 0 ? `Target group: ${groups[st.TargetGroupId].children?.name}` : null,
        st.TargetUnitId >= 0 ? `Target unit: ${units[st.TargetUnitId].children?.name}` : null
        ].filter(Boolean);
        if (infoLines.length) label += '\n' + infoLines.join('\n');
    }
    return {
        data: {
            id: n.id,
            label,
            parent: n.parentChain?.at(-1),
            type: n.type,
            state: n.state,
            action: n.action,
            parameters: n.parameters,
            isStartBehavior: !!n.isStartBehavior
        }
    };
    });


    // Setup edges
    const cyEdges = edges.map(e => ({
        data: {
            id: e.id,
            source: e.source,
            target: e.target,
            type: getConcreteType(e.raw?.type),
            isMutuallyExclusive: e.mutuallyExclusive ? "true" : "false",
            label: getConcreteType(e.raw?.type) === "TDuration" ? "After " + e.raw?.children.Duration + "ms" : getConcreteType(e.raw?.type) === "TTimestamp" ? "At time: " + e.raw?.children.Timestamp : ""
        }
    }));

    return {cyNodes, cyEdges, startBehaviors, isRunning};
}
