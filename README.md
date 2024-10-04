# Extractor
This node subscribes to image and audio data, merging messages with similar timestamps into a new message published on `/extractor/av_message`

## Build
To build the project, use the following command. This avoids potential dependency errors:

`colcon build --merge-install --symlink-install --cmake-args -Wall -Wextra -Wpedantic`

## Run
run the extractor with `ros2 run extractor_node extract`

