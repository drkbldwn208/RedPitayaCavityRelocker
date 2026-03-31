#!/bin/bash

# Check if an input file was provided
if [ -z "$1" ]; then
    echo "Usage: $0 <path_to_xdc_file>"
    exit 1
fi

INPUT_XDC="$1"
OUTPUT_TCL="create_bd_ports.tcl"

echo "Scraping $INPUT_XDC for port definitions..."

awk '
/get_ports/ {
    # Extract the base port name using match and substr for pure POSIX compatibility
    match($0, /get_ports \{?[a-zA-Z0-9_]+/)
    if (RSTART > 0) {
        raw_port = substr($0, RSTART, RLENGTH)
        # Strip the "get_ports {" prefix to isolate the name
        sub(/get_ports \{?/, "", raw_port)
        port = raw_port

        # Infer direction based on standard Red Pitaya suffix
        if (port ~ /_o$/) dirs[port] = "O"
        else if (port ~ /_i$/) dirs[port] = "I"
        else dirs[port] = "IO" # Fallback if no suffix is found

        # Flag as a vector if it contains [*]
        if ($0 ~ /\[\*\]/) is_vector[port] = 1

        # Check for specific pin indices (e.g., [15]) to find the maximum bit width
        match($0, /\[[0-9]+\]/)
        if (RSTART > 0) {
            is_vector[port] = 1
            idx_str = substr($0, RSTART, RLENGTH)
            gsub(/\[|\]/, "", idx_str) # Strip brackets
            idx_val = idx_str + 0      # Convert to integer
            
            # Keep track of the highest index seen for this port
            if (!(port in max_idx) || idx_val > max_idx[port]) {
                max_idx[port] = idx_val
            }
        }
    }
}
END {
    print "# Auto-generated TCL script to create BD ports"
    for (p in dirs) {
        if (is_vector[p] == 1 && p in max_idx) {
            printf "create_bd_port -dir %s -from %d -to 0 %s\n", dirs[p], max_idx[p], p
        } else {
            printf "create_bd_port -dir %s %s\n", dirs[p], p
        }
    }
}
' "$INPUT_XDC" > "$OUTPUT_TCL"

echo "Done! Generated TCL script: $OUTPUT_TCL"
