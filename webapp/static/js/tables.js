function unifyProperties(objects) {

    function collectProperties(object, unifiedObject, isTopLevel) {
        // Emit an id column only when it is meaningful:
        //  - top-level objects: always (this outer id is the relation target)
        //  - nested objects: only if they carry a real inner id in children
        const hasInnerId = object.children && object.children.id !== undefined;
        if ((isTopLevel || hasInnerId) &&
            object.id !== undefined &&
            !unifiedObject.children.find(c => c.name === "id")) {
            unifiedObject.children.unshift({ name: "id", children: [] });
        }

        for (let key in object.children) {
            if (!object.children.hasOwnProperty(key)) continue;
            if (key === "id") continue; // handled by the id column above

            if (typeof object.children[key] === "object" && object.children[key].children === undefined) {
                //empty object -> ignore
                continue;
            }

            let existingChild = unifiedObject.children.find(child => child.name === key);
            if (!existingChild) {
                existingChild = { name: key, children: [] };
                unifiedObject.children.push(existingChild);
            }

            if (typeof object.children[key] === "object" && object.children[key] !== null) {
                collectProperties(object.children[key], existingChild, false);
            }
        }
    }

    const unified = { name: "root", children: [] };

    objects.forEach(object => {
        collectProperties(object, unified, true); // top-level entries
    });

    return unified.children;
}


function generateMultiRowHeader(properties) {
    const headerRows = [];

    function processObject(property, currentPath, currentPosition) {
        if (!headerRows[currentPath.length]) {
            headerRows[currentPath.length] = [];
        }

        //add this property to currentPath
        currentPath.push(property.name);

        headerRows[currentPath.length - 1].push({
            path: [...currentPath],
            colspan: getColspan(property),
            position: currentPosition.pos,
            primitive: property.children.length === 0
        });

        for (const i in property.children) {
            processObject(property.children[i], currentPath, currentPosition);
        }

        //remove this property from currentPath
        currentPath.pop();

        if (property.children.length === 0) {
            currentPosition.pos++;
        }
    }

    function getColspan(property) {
        let colspan = 0;
        for (const i in property.children) {
            colspan += getColspan(property.children[i]);
        }
        if (colspan === 0) {
            colspan = 1;
        }
        return colspan;
    }

    let currentPosition = {pos: 0};
    for (const i in properties) {
        processObject(properties[i], [], currentPosition);
    }

    return headerRows;
}

function getPropertyValue(object, path) {
    if (path.length === 0) {
        if (typeof object === "number" && !Number.isInteger(object)) {
            if (config_round_decimals >= 0) {
                return Number(object.toFixed(config_round_decimals));
            }
        }
        return object;
    }

    let newPath = [...path];
    let nextStep = newPath.shift();

    // "id" columns refer to the object's own (non-nested) id
    if (nextStep === "id" && newPath.length === 0) {
        if (object.children && object.children.id !== undefined) return object.children.id; // inner/domain id
            return object.id !== undefined ? object.id : "";                                    // fall back to outer id
    }



    if (object.children.hasOwnProperty(nextStep)) {
        return getPropertyValue(object.children[nextStep], newPath);
    } else {
        return "";
    }
}


function calculateWidthOfHeader(headerRows) {
    return headerRows.reduce((maxSum, columns) => {
        const sum = columns.reduce((acc, obj) => {
            return acc + (Number(obj.colspan) || 0);
        }, 0);

        return Math.max(maxSum, sum);
    }, 0);
}

function generateTable(componentsName, components) {

    let rootProperties = unifyProperties(components);
    if (components.length === 0 || rootProperties.length === 0) {
        return
    }

    let table = document.createElement("table");
    table.className = "styled-table"

    let caption = table.createCaption();
    caption.textContent = componentsName;
    caption.className = "table-caption";

    let thead = document.createElement("thead");

    let headerRows = generateMultiRowHeader(rootProperties)
    let width = calculateWidthOfHeader(headerRows)

    let columns = [];
    for (let level in headerRows) {
        let row = document.createElement("tr");
        let currentPos = 0;
        for (let element in headerRows[level]) {
            let property = headerRows[level][element];
            let targetPos = property.position;
            if (targetPos > currentPos) {
                let filler = document.createElement("th");
                filler.colSpan = targetPos - currentPos;
                row.appendChild(filler)
                currentPos = targetPos;
            }
            let th = document.createElement("th");
            th.textContent = property.path[property.path.length - 1];

            th.colSpan = property.colspan;
            row.appendChild(th);
            
            if (property.primitive) {
                if(columns.length < currentPos) {
                    while(columns.length < currentPos) {
                        columns.push(undefined);
                    }
                    columns.push(property.path);
                } else {
                    columns[currentPos] = property.path;
                }
            }

            currentPos += property.colspan;
        }

        if(width > currentPos) {
            let filler = document.createElement("th");
            filler.colSpan = width - currentPos;
            row.appendChild(filler);
        }

        thead.appendChild(row);
    }

    let tbody = document.createElement("tbody");

    components.forEach(component => {
        let row = document.createElement("tr");
        columns.forEach(column => {
            let value = getPropertyValue(component, column);
            let td = document.createElement("td");
            td.textContent = value;
            row.appendChild(td);
        });
        tbody.appendChild(row);
    });

    table.appendChild(thead);
    table.appendChild(tbody);

    document.getElementById("table-container").appendChild(table);
}
