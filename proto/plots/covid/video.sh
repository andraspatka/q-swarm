#!/bin/bash
# ================================================================================
# ================================================================================

# Subtasks implemented as functions


# Capture subtask.
function f_capture {
    frames=$(ls frames | egrep "^frame_[0-9]{5}.png")
    starting_index=$(echo ${frames} | tr " " "\n" | head -1 | tr -d "frame_" | tr -d ".png")
    ffmpeg -start_number ${starting_index} -framerate 24 -i frames/frame_%05d.png \
        -vf scale=1280:-2 -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p \
        captures/${cap_output}.mp4
}

# ================================================================================
if (( $# < 1 || $# > 1 )); then # Invalid number of arguments
	echo "Invalid usage. Correct usage: video cap_output"
	exit 1;
fi

cap_output=$1
if [[ -z ${cap_output} ]]; then
    echo "Invalid usage. Correct usage: qswarm capture output_file_name";
    exit 1;
fi

f_capture

exit 0;