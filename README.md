# Extractor and Processor
This extractor node subscribes to image and audio data, merging messages with similar timestamps into a new message published on `/extractor/av_message`

While processor node subscribes to the `/extractor/av_message` message, then it will publish the beam_overlay image which shows the source of the sound.

## Build
To build the project, use the following command. This avoids potential dependency errors:

`colcon build --merge-install --symlink-install --cmake-args -Wall -Wextra -Wpedantic`

## Run
launch the extractor and processor with `ros2 launch processor processor.launch.py `


