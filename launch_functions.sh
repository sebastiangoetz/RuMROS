#!/bin/bash

# Helper to set the terminal title
set-title() {
  if [[ -z "$ORIG" ]]; then
    ORIG=$PS1
  fi
  TITLE="\[\e]2;$*\a\]"
  PS1=${ORIG}${TITLE}
}

# To terminate everything from the initial terminal via ctrl+c
PGIDS=()
cleanup() {
    echo "Caught SIGINT. Killing application terminals..."
    for pgid in "${PGIDS[@]}"; do
        echo "Killing process group $pgid"
        kill -- -"$pgid" 2>/dev/null
    done
    exit 0
}

# Launches terminal and records PGID
launch_terminal() {
    local cmd="$1"
    local title="$2"

    gnome-terminal -- bash -c "
        echo \"Launched PID \$BASHPID (PGID \$(ps -o pgid= -p \$BASHPID))\"

        # Set terminal title if provided
        if [[ -n \"$title\" ]]; then
            echo -ne \"\033]0;$title\007\"
        fi

        $cmd
        exec bash
    " &

    # Wait briefly for the terminal to launch
    sleep 1

    # Capture the PGID of the most recently spawned bash terminal
    PGID=$(ps -eo pgid,pid,cmd | grep '[b]ash -c' | sort -k2 -nr | head -1 | awk '{print $1}')
    PGIDS+=("$PGID")
}

