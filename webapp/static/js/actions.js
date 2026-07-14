function validateNumberInput(text) {
    //Keep only digits, dot, and minus sign
    return text
        .replace(/[^0-9.-]/g, "")        //remove invalid chars
        .replace(/(?!^)-/g, "")          //remove minus if not at start
        .replace(/(\..*?)\..*/g, "$1");  //allow only one decimal point
}

function createOrUpdateForm(action, index, typeMap, disableButtons) {
    const actionId = action.children.id;
    const actionContainer = document.getElementById("action-container");

    //create form with header if it doesn't exist
    const form = actionContainer.querySelector("#" + actionId) || (() => {
        let newForm = document.createElement("form");
        newForm.id = actionId;

        let headingWrapper = document.createElement("div");
        headingWrapper.style.display = "flex";
        headingWrapper.style.alignItems = "center";
        headingWrapper.style.gap = "8px";
        headingWrapper.style.justifyContent = "space-between";


        let heading = document.createElement("h3");
        heading.innerText = action.children.label;
        headingWrapper.appendChild(heading);

        // Checkbox + label
        let checkboxLabel = document.createElement("label");
        checkboxLabel.style.fontSize = "0.9em";
        checkboxLabel.style.display = "flex";
        checkboxLabel.style.alignItems = "center";
        checkboxLabel.style.gap = "4px";

        let defaultCheckbox = document.createElement("input");
        defaultCheckbox.type = "checkbox";
        defaultCheckbox.className = "default-checkbox";

        checkboxLabel.appendChild(defaultCheckbox);
        checkboxLabel.appendChild(document.createTextNode("Use default parameters"));
        headingWrapper.appendChild(checkboxLabel);

        newForm.appendChild(headingWrapper);

        //handle default checkbox
        defaultCheckbox.addEventListener("change", () => {
            const rows = newForm.querySelectorAll("tr.form-group");
            rows.forEach(row => {
                const select = row.querySelector("select");
                const input = row.querySelector("input");

                if (!select) {
                    //hide only if it is an input with a placeholder (i.e. default value)
                    if (input && input.hasAttribute("placeholder") && input.getAttribute("placeholder").trim() !== "") {
                        row.style.display = defaultCheckbox.checked ? "none" : "";
                    } else {
                        row.style.display = "";
                    }
                }
            });
        });


        actionContainer.appendChild(newForm);
        return newForm;
    })();

    //create table if it doesn't exist
    const table = form.querySelector("table") || (() => {
        let newTable = document.createElement("table");
        newTable.className = "action-table";
        form.appendChild(newTable);
        return newTable;
    })();

    action.children.Input.forEach(inputObject => {
        const inputType = inputObject.children.type.split('.').pop();
        //create row if it doesn't exist
        const row = table.querySelector("#" + inputObject.children.id) || (() => {
            let newRow = document.createElement("tr");
            newRow.className = "form-group";
            newRow.id = inputObject.children.id;
            table.appendChild(newRow);
            return newRow;
        })();

        //create td and label if it doesn't exist
        const label = row.querySelector(".action-label") || (() => {
            let labelTd = document.createElement("td");
            const newLabel = document.createElement("label");
            newLabel.innerText = inputObject.children.label;
            newLabel.className = "action-label";
            labelTd.appendChild(newLabel);
            row.appendChild(labelTd);
            return newLabel;
        })();

        //create td for input if it doesn't exist
        const inputTd = row.querySelector(".input-td") || (() => {
            let newInputTd = document.createElement("td");
            newInputTd.className = "input-td";
            row.appendChild(newInputTd);
            return newInputTd;
        })();

        if (inputTd.querySelector("input, select") == null) {
            if (inputType === "String") {
                const input = document.createElement("input");
                input.name = inputObject.children.id;
                input.type = "text";
                input.className = "action-input";
                input.placeholder = inputObject.children.defaultValue;
                input.style.width = "100px";
                inputTd.appendChild(input);
            } else if (inputType === "Integer") {
                const input = document.createElement("input");
                input.name = inputObject.children.id;
                input.type = "number";
                input.step = "1";
                let num = parseInt(inputObject.children.defaultValue);
                if (num)
                    input.placeholder = num;
                input.className = "action-input";
                input.style.width = "100px";
                input.addEventListener("input", function () {
                    this.value = validateNumberInput(this.value);
                });
                inputTd.appendChild(input);
            } else if (inputType === "Double") {
                const input = document.createElement("input");
                input.name = inputObject.children.id;
                input.type = "text";
                input.className = "action-input";
                let num = parseFloat(inputObject.children.defaultValue);
                if (num)
                    input.placeholder = num;
                input.style.width = "100px";
                input.addEventListener("input", function () {
                    this.value = validateNumberInput(this.value);
                });
                inputTd.appendChild(input);
            } else if (inputType === "Boolean") {
                const input = document.createElement("input");
                input.name = inputObject.children.id;
                input.type = "checkbox";
                input.className = "action-input";
                input.checked = stringToBoolean(inputObject.children.defaultValue);
                //input.style.width = "20px";
                inputTd.appendChild(input);
            } else if (typeMap[inputType] !== undefined) {
                const select = document.createElement("select");
                select.name = inputObject.children.id;
                select.className = "action-input";

                Object.entries(typeMap[inputType]).forEach(([id, name]) => {
                    const option = document.createElement("option");
                    option.id = id;
                    option.value = id;
                    option.innerText = name;
                    select.appendChild(option);
                });

                inputTd.appendChild(select);
            }
        } else {
            if (typeMap[inputType] !== undefined) {
                const select = inputTd.querySelector("select");

                //create a Set of the current option IDs in the select
                let existingIds = new Set();
                for (let option of select.options) {
                    existingIds.add(option.value);
                }

                //loop through the new map and update existing options or add new ones
                Object.entries(typeMap[inputType]).forEach(([id, name]) => {
                    let option = select.querySelector("option[value=\"" + id + "\"]");

                    if (option) {
                        //if the option exists but the name has changed, update it
                        if (option.innerText !== name) {
                            option.innerText = name;
                        }
                    } else {
                        //if the option doesn't exist, create a new one
                        let newOption = document.createElement("option");
                        newOption.value = id;
                        newOption.textContent = name;
                        select.appendChild(newOption);
                    }

                    // Remove this id from the existingIds set since it's accounted for
                    existingIds.delete(id);
                });

                //remove options that are not in the new map
                for (let id of existingIds) {
                    let optionToRemove = select.getElementById(id);
                    if (optionToRemove) {
                        select.removeChild(optionToRemove);
                    }
                }
            }
        }
    });

    const startButton = table.querySelector("button") || (() => {
        const row = document.createElement("tr");
        const td = document.createElement("td");
        td.colSpan = 2;

        // Create the start button
        const newStartButton = document.createElement("button");
        newStartButton.type = "button";
        newStartButton.innerText = "Start";
        newStartButton.className = "goal-button"
        newStartButton.addEventListener("click", () => {
            let inputs = getFormValues(form);
            const actionData = { id: actionId, inputs };
            socket.emit("runAction", actionData);
        });

        td.appendChild(newStartButton);
        row.appendChild(td);
        table.appendChild(row);

        return newStartButton;
    })();

    startButton.disabled = disableButtons;

    const result = action.children.Result;

    if (result.children.type !== "EMPTY" && startButton) {

        const beforeText = startButton.innerText;
        const beforeStyle = startButton.style;

        //show action result
        startButton.innerText = result.children.message;
        let color;
        switch (result.children.type) {
            case "SUCCESS":
                color = "green";
                break;
            case "ERROR":
                color = "red";
                break;
            case "TECHNICAL_ERROR":
                color = "black";
                break;
        }
        startButton.style.color = color;
        startButton.style.backgroundColor = "transparent";
        startButton.style.border = "none";
        startButton.style.cursor = "default";
        startButton.disabled = true;

        if (result.children.type !== "WAIT") {
            if (!disableButtons) {
                //reset after some time
                setTimeout(() => {
                    startButton.innerText = beforeText;
                    startButton.style = beforeStyle;
                    startButton.disabled = false;
                }, result.children.displayMillis);
            }
        } else {
            //defer reset to next updateSensorData call
            pendingButtonReset = () => {
                startButton.innerText = beforeText;
                startButton.style = beforeStyle;
                startButton.disabled = false;
                pendingButtonReset = null; //clear after running
            };
        }
    }
}

function getFormValues(form) {
    let formData = {};

    let inputs = form.querySelectorAll("input, select");

    inputs.forEach(input => {
        if (input.type === "checkbox")
            formData[input.name] = input.checked;
        else {
            //All other tags: use placeholder or value
            //Replace empty values with placeholder values
            if (input.value.trim() === "" && input.placeholder) {
                formData[input.name] = input.placeholder;
            } else
                formData[input.name] = input.value;
        }
    });

    return formData;
}