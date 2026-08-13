// --- Cookie helpers ---
function getCookie(name) {
    const match = document.cookie.match(new RegExp('(?:^|; )' + name + '=([^;]*)'));
    return match ? decodeURIComponent(match[1]) : null;
}

function setCookie(name, value, days = 365) {
    const expires = new Date(Date.now() + days * 864e5).toUTCString();
    document.cookie = `${name}=${encodeURIComponent(value)}; expires=${expires}; path=/; SameSite=Lax`;
}

// Objects hidden from tables by default (previously hard-coded in processData)
const DEFAULT_HIDDEN_OBJECTS = [];
const HIDDEN_OBJECTS_COOKIE = "hiddenObjects";

// User-added hidden object keys (persisted via cookie)
function getUserHiddenObjects() {
    const stored = getCookie(HIDDEN_OBJECTS_COOKIE);
    if (!stored) return [];
    try {
        const parsed = JSON.parse(stored);
        return Array.isArray(parsed) ? parsed : [];
    } catch (e) {
        return [];
    }
}

function setUserHiddenObjects(list) {
    setCookie(HIDDEN_OBJECTS_COOKIE, JSON.stringify(list));
}

// Combined set of everything that should be removed from the tables
function getAllHiddenObjects() {
    return new Set([...DEFAULT_HIDDEN_OBJECTS, ...getUserHiddenObjects()]);
}


// Required for awaitable results
let pendingButtonReset = null;

// Color helpers
function hexToRgb(hex) {
    // Credit: https://stackoverflow.com/a/5624139
    // Expand shorthand form (e.g. "03F") to full form (e.g. "0033FF")
    var shorthandRegex = /^#?([a-f\d])([a-f\d])([a-f\d])$/i;
    hex = hex.replace(shorthandRegex, function(m, r, g, b) {
        return r + r + g + g + b + b;
    });

    var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? {
        r: parseInt(result[1], 16),
        g: parseInt(result[2], 16),
        b: parseInt(result[3], 16)
    } : null;
}

function getColorVariableRGB(varName) {
    return hexToRgb(window.getComputedStyle(document.body).getPropertyValue(varName));
}

function getColorStringRGBA(rgbColor, opacity = 1.0) {
    return "rgba(" + rgbColor + "," + opacity + ")";
}

// Shortcut
function getColorByVariable(varName, opacity = 1.0) {
    let colArray = getColorVariableRGB(varName);
    let colStrRGB = [colArray.r, colArray.g, colArray.b].join(",");
    return getColorStringRGBA(colStrRGB, opacity);
}


// Setup global colors
const colAccent = getColorByVariable("--col-accent");
const colAccentLight = getColorByVariable("--col-accent-light");
const colAccentLightMuted = getColorByVariable("--col-accent-light-muted");

const colText = getColorByVariable("--col-text");
const colWarning = getColorByVariable("--col-warning");
const colInactive = getColorByVariable("--col-inactive");
const colInactiveLight = getColorByVariable("--col-inactive-light");
const colInactiveDark = getColorByVariable("--col-inactive-dark");
const colMutuallyExclusive = getColorByVariable("--col-mutually-exclusive");
const colEmphasized = getColorByVariable("--col-emphasized");
const colEmphasizedMid = getColorByVariable("--col-emphasized-mid");
const colEmphasizedLight = getColorByVariable("--col-emphasized-light");


// Init socket
// IAI binder: socket.io has to work on subpage as well
const socket = io(window.location.origin, {
    path: `${SCRIPT_ROOT}/socket.io/`
});
socket.on("updateSensorData", function (msg) {
    if (pendingButtonReset) {
        pendingButtonReset(); //apply delayed reset for awaitable results
    }
    
    let data = JSON.parse(msg);
    let dataOrig = structuredClone(data); // Deep copy (processData removes some parts)
    processData(data, true);

    //Trigger update event, may be used by other scripts to react to changes
    document.dispatchEvent(new CustomEvent("contentUpdated", {detail: dataOrig}));
});

function objectsToLists(objects) {
    //convert single components into lists that contain only the single component for easier handling
    return Object.entries(objects).map(([key, value]) => {
        return [key, Array.isArray(value) ? value : [value]];
    });
}

function objectListsToMap(objectEntries) {
    //find all existing objects
    let objectsMap = {};
    objectEntries.forEach(([, components]) => {
        components.forEach(component => {
            collectObjects(component, objectsMap);
        })
    });

    //resolve relations into readable references
    objectEntries.forEach(([, components]) => {
        components.forEach(component => {
            expandObject(component, objectsMap);
        })
    });

    return objectsMap;
}


function isInClassList(listString, value) {
    if (!listString) return false;
    const parts = listString.split("|").map(s => s.trim());
    return parts.includes(value);
}

function getConcreteType(listString) {
    if (!listString) return false;
    const parts = listString.split("|").map(s => s.trim());
    if (parts.length > 0) return parts[0];
    return "";
}

function getAllObjectsofType(objectsMap, type) {
    let result = [];
    let seen = new Set();

    function traverse(obj) {
        if (!obj || typeof obj !== "object") return;

        if (obj.type && isInClassList(obj.type, type)) {
            if (!seen.has(obj.id)) {
                seen.add(obj.id);
                result.push(obj);
            }
        }

        if (Array.isArray(obj)) {
            for (let item of obj) traverse(item);
        } else {
            for (let key in obj) {
                traverse(obj[key]);
            }
        }
    }

    traverse(objectsMap);
    return result;
}

function getObjectsOfType(input, typeName) {
    const results = [];
    const seen = new WeakSet();

    function add(obj) {
        if (obj && typeof obj === 'object' && !seen.has(obj)) {
            seen.add(obj);
            results.push(obj);
        }
    }

    // Normalize input -> list of "container" entries
    let containers = [];
    if (Array.isArray(input)) {
        containers = input;
    } else if (input && typeof input === 'object') {
        const vals = Object.values(input);
        // Heuristic: if the object's values look like typed entries, treat input as a root map
        const looksLikeRoot = vals.length > 0 && vals.every(v => v && typeof v === 'object' && 'type' in v);
        containers = looksLikeRoot ? vals : [input];
    } else {
        return results;
    }

    // For each container (top-level entry), collect matches at:
    // 1) container itself
    // 2) container direct properties (arrays / objects)
    // 3) container->prop (if object) -> its direct array properties (one more level)
    containers.forEach(container => {
        if (!container || typeof container !== 'object') return;

        // 1) container itself
        if (isInClassList(container.type, typeName)) add(container);

        // iterate all property values of the container
        Object.values(container).forEach(propVal => {
            if (!propVal) return;

            // 2a) if property is an array, check its items
            if (Array.isArray(propVal)) {
                propVal.forEach(item => {
                    if (isInClassList(item.type, typeName)) add(item);
                });
            }

            // 2b) if property is an object, check it and one level deeper inside it
            else if (typeof propVal === 'object') {
                if (isInClassList(propVal.type, typeName)) add(propVal);

                // scan the object's own values for arrays/objects (one level deeper)
                Object.values(propVal).forEach(nested => {
                    if (!nested) return;
                    if (Array.isArray(nested)) {
                        nested.forEach(item => {
                            if (isInClassList(item.type, typeName)) add(item);
                        });
                    } else if (nested && typeof nested === 'object') {
                        if (isInClassList(nested.type, typeName)) add(nested);
                    }
                });
            }
        });
    });

    return results;
}

function processData(objects, removeNoTableObjects = false) {
    let tableContainer = document.getElementById("table-container");
    if (tableContainer != null)
        tableContainer.innerHTML = "";

    let behaviorModelRunning = false;
    if (objects.BehaviorModel)
        behaviorModelRunning = objects.BehaviorModel.children?.running;

    let actions = objects["Action"];
    //remove unwanted tables (defaults + user-configured via settings cog)
    if (removeNoTableObjects) {
        getAllHiddenObjects().forEach(key => {
            if (objects.hasOwnProperty(key))
                delete objects[key];
        });
    }
        

    //convert single components into lists that contain only the single component for easier handling
    let objectEntries = objectsToLists(objects)

    //find all existing objects
    let objectsMap = objectListsToMap(objectEntries);

    //remove additional, because they are no longer needed
    delete objects.Additional;

    if (tableContainer != null) {
        objectEntries.forEach(([componentsName, components]) => {
            generateTable(componentsName, components);
        });
    }

    //create typeMap
    let typeMap = {};
    Object.values(objectsMap).forEach(object => {
        if (object.children === undefined || object.children.name === undefined) {
            //skip empty objects or objects that don't have a name
            return;
        }

        const types = object.type.split('|');
        for (const type of types) {
            if (!(type in typeMap)) {
                typeMap[type] = {};
            }
            typeMap[type][object.id] = object.children.name;
        }
    })

    let actionContainer = document.getElementById("action-container");
    if (actionContainer != null) {
        for (let i in actions) {
            createOrUpdateForm(actions[i], i, typeMap, behaviorModelRunning);
        }
    }
}

function collectObjects(object, objectsMap) {
    objectsMap[object.id] = object;
    for (let childKey in object.children) {
        if (object.children.hasOwnProperty(childKey)) {

            if (typeof object.children[childKey] === "object" && object.children[childKey] !== null) {
                collectObjects(object.children[childKey], objectsMap);
            }
        }
    }
}

function resolveReference(map, id) {
    let object = map[id];
    if (!object) {
        console.error("Missing related object with id " + id);
        return id;
    }

    let c = object.children;
    if (c != null) {
        //prefer a human-readable name
        if (c.name !== undefined) {
            return c.name;
        }
        //fall back to the object's true internal id
        if (c.id !== undefined) {
            return c.id;
        }
    }

    return id;
}


function expandObject(object, map) {    
    if (object.hasOwnProperty("relations")) {
        Object.entries(object.relations).forEach(([key, target]) => {
            if (Array.isArray(target)) {
                //Relation to multiple objects -> comma-separated list of references
                object.children[key] = target
                    .map(id => resolveReference(map, id))
                    .join(", ");
            } else {
                //Relation to a single object -> single reference
                object.children[key] = resolveReference(map, target);
            }
        });
        delete object.relations;
    }

    for (let key in object.children) {
        if (typeof object.children[key] === "object" && object.children[key] !== null) {
            expandObject(object.children[key], map);
        }
    }
}


function resolveRelation(object, map, alreadyVisited, key, objectId) {
    if (!alreadyVisited.has(objectId)) {
        if (!map.hasOwnProperty(objectId)) {
            console.error("Missing relative object of class " + key + " with id " + objectId)
            return;
        }
        object.children[key] = map[objectId];
        alreadyVisited.add(objectId);
    } else {
        //skip
    }
}

function resolveInternalId(map, id) {
    let object = map[id];
    if (object.children != null) {
        let c = object.children;
        if (c.id == parseInt(c.id, 10)) {
            return c.id;
        }
    }
    console.error("Error retrieving object's true ID from web UI id " + id);
    return null;
}

function collectBehaviors(node, out = []) {
    if (!node) return out;
    if (node.type && (node.type.includes("CB") || node.type.includes("BB"))) {
        out.push(node);
        const subs = node.children?.Behavior || [];
        subs.forEach(sb => collectBehaviors(sb, out));
    }
    return out;
}

function isType(obj, typeName) {
    return obj.type && obj.type.includes(typeName);
}

//https://stackoverflow.com/a/1414175
const stringToBoolean = (stringValue) => {
    switch(stringValue?.toLowerCase()?.trim()){
        case "true": 
        case "yes": 
        case "1": 
            return true;

        default: 
            return false;
    }
}

function parseIntOrDefault(value, fallback) {
    if (value == null)
        return fallback;
    const num = parseInt(value);
    return isNaN(num) ? fallback : num;
}

function parseFloatOrDefault(value, fallback) {
    if (value == null)
        return fallback;
    const num = parseFloat(value);
    return isNaN(num) ? fallback : num;
}