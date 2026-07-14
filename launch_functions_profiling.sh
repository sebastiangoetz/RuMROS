#!/bin/bash

# Helper to set the terminal title
set-title() {
  if [[ -z "$ORIG" ]]; then
    ORIG=$PS1
  fi
  TITLE="\[\e]2;$*\a\]"
  PS1=${ORIG}${TITLE}
}

# Arrays to track process groups and loggers
PGIDS=()
MONITORS=()

# Log to files in group folder
LOG_FOLDER="perflogs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_FOLDER}"

cleanup() {
    echo "Caught SIGINT. Killing application terminals and monitors..."
    for pid in "${MONITORS[@]}"; do
        echo "Stopping monitor PID $pid"
        kill "$pid" 2>/dev/null
    done
    for pgid in "${PGIDS[@]}"; do
        echo "Killing process group $pgid"
        kill -- -"$pgid" 2>/dev/null
    done
    exit 0
}

# Start performance logging for a given process group
start_perf_monitor() {
    local pgid="$1"
    local title="$2"
    local logfile="${LOG_FOLDER}/${pgid}_${title// /_}.log"

    echo "Starting performance monitor for PGID $pgid > $logfile"
    (
        #echo "timestamp,PID,CPU%,MEM%,RSS(KB),COMMAND"
        while true; do
            timestamp=$(date +"%Y-%m-%d %H:%M:%S")
            echo "$timestamp" >> "$logfile"
            ps -eo pgid,pid,pcpu,pmem,rss,cmd \
                | awk -v pgid="$pgid" '$1 == pgid {print "  " $2, $3, $4, $5, $6, $7, $8, $9, $10}' >> "$logfile"
            sleep 1 # Log every second
        done
    ) &
    MONITORS+=($!)
}

# Launch terminal and record PGID
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

    # Wait for the terminal to launch
    sleep 1

    # Capture the PGID of the most recently spawned bash terminal
    PGID=$(ps -eo pgid,pid,cmd | grep '[b]ash -c' | sort -k2 -nr | head -1 | awk '{print $1}')
    PGIDS+=("$PGID")

    # Start performance monitoring
    start_perf_monitor "$PGID" "$title"
}
