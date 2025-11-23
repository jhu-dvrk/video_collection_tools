# JHU Data Collection

A GStreamer-based multi-stream video player, recorder, and frame extraction toolkit with a GTK3 GUI, written in C++ using CMake.

## Features

### Video Recorder

- **Multi-Stream Support**: Configurable via `config.json` to handle multiple video sources simultaneously.
- **Central Control Panel**: A dedicated "Controls" window to manage all video streams with integrated log viewer.
- **Selective Recording**:
  - Master "Record" switch.
  - Individual stream selection for recording (per-source defaults configurable).
  - Records to **MP4** format (H.264 video).
- **Metadata Generation**:
  - Generates a sidecar `.json` file for each video recording.
  - Contains start/stop timestamps with nanosecond precision (both human-readable and `timespec` format), frame counts, PTS/DTS, and average FPS.
- **Dynamic Data Directory**:
  - Select output directory via the UI.
  - Automatically creates directories if they don't exist.
  - Automatically restarts active recordings when the directory is changed.
- **Hardware Acceleration**: Uses `glimagesink` for efficient rendering.
- **Flexible Video Preview**:
  - Right-click context menu to toggle aspect ratio constraints.
  - Optional secondary `glimagesink` output per source (configurable via `tee_gl_view` flag).
- **Live Log Monitoring**: Scrollable text view displaying application messages in real-time.

### Frame Extractor

- **Precise Frame Extraction**: Extracts individual frames from recorded MP4 videos.
- **Nanosecond-Precision Timestamps**: Calculates frame timestamps using the precise start time from recording metadata.
- **Automatic Metadata Generation**: Creates an `index.json` file containing frame-level metadata.
- **PNG Output**: Saves each frame as a numbered PNG file.
- **Simple Interface**: Single parameter invocation using the metadata JSON file.

## Dependencies

- CMake (>= 3.10)
- C++17 compiler
- GStreamer 1.0 (Core, Video, GL, Plugins)
- GTK3
- JsonCpp

### Installing Dependencies

#### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install cmake build-essential
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install gstreamer1.0-plugins-base gstreamer1.0-plugins-good
sudo apt-get install gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
sudo apt-get install gstreamer1.0-libav gstreamer1.0-tools
sudo apt-get install gstreamer1.0-gl gstreamer1.0-plugins-base-apps
sudo apt-get install libgtk-3-dev libjsoncpp-dev
```

## Configuration

The application reads from a `config.json` file in the working directory.

Example `config.json`:
```json
{
    "data_directory": "data",
    "sources": [
        {
            "name": "Camera 1",
            "stream": "videotestsrc pattern=ball",
            "record": true,
            "tee_gl_view": false
        },
        {
            "name": "Camera 2",
            "stream": "videotestsrc pattern=smpte",
            "record": false,
            "tee_gl_view": true
        }
    ]
}
```

### Configuration Fields

- **`data_directory`**: Output directory for recordings (relative or absolute path). Created automatically if it doesn't exist.
- **`sources`**: Array of video source configurations.
  - **`name`**: Display name for the source.
  - **`stream`**: GStreamer pipeline description for the video source.
  - **`record`**: Boolean indicating whether this source should be selected for recording by default.
  - **`tee_gl_view`**: Boolean to enable an additional `glimagesink` window for this source (useful for debugging or multi-monitor setups).


## Building (ROS2 / Colcon)

1. From the root of your ROS2 workspace (e.g., `~/ros2_ws`), clone this repository into the `src` folder:
  ```bash
  cd ~/ros2_ws/src
  git clone <repo-url>
  ```

2. Install dependencies (see above for system packages).

3. Build the workspace using colcon:
  ```bash
  cd ~/ros2_ws
  colcon build --packages-select jhu_data_collection
  ```

4. Source the workspace:
  ```bash
  source install/setup.bash
  ```

This builds the following executables:
- `video_recorder`: The main recording application (without ROS topics)
- `video_recorder_node`: Same as above but with ROS topics to start/stop collection
- `frame_extractor`: Frame extraction tool (ROS independent)


## Running

### Video Recorder

Ensure `config.json` is in the current directory (or the build directory if running from there).

```bash
./video_recorder -c config.json
```

### Video Recorder Node

```bash
ros2 run jhu_data_collection video_recorder_node -c <path_to_config_file>
```

### Frame Extractor

Extract frames from a recorded video by providing the metadata JSON file:

```bash
./frame_extractor /path/to/Camera_1_2025-11-19_14-30-00.mp4.json
```

The extractor will:
1. Read the metadata JSON file to find the corresponding MP4 file
2. Create an output directory named `{video_basename}_frames/`
3. Extract all frames as `frame_000001.png`, `frame_000002.png`, etc.
4. Generate an `index.json` file with per-frame metadata including precise timestamps

### ROS2 Python Control Scripts

The following ROS2 Python scripts are provided for controlling the video recorder node:

- `start_recording.py`: Publishes a message to `/video_recorder/control` to start recording.
- `stop_recording.py`: Publishes a message to `/video_recorder/control` to stop recording.
- `set_data_directory.py <directory_path>`: Publishes the new data directory to `/video_recorder/data_directory`.
- `set_recording.py <pipeline_name>`: Publishes a message to `set_record` to select a pipeline for recording.
- `unset_recording.py <pipeline_name>`: Publishes a message to `unset_record` to deselect a pipeline for recording.

Example usage:

```bash
ros2 run jhu_data_collection start_recording.py
ros2 run jhu_data_collection stop_recording.py
ros2 run jhu_data_collection set_data_directory.py /path/to/data
ros2 run jhu_data_collection set_recording.py Camera_1
ros2 run jhu_data_collection unset_recording.py Camera_1
```

## Output

### Video Recorder Output

Recorded files are saved to the configured data directory with the naming convention:
`{Source_Name}_{YYYY-MM-DD_HH-MM-SS}.mp4`

A corresponding JSON metadata file is created:
`{Source_Name}_{YYYY-MM-DD_HH-MM-SS}.mp4.json`

Example Metadata:
```json
{
    "filename": "Camera_1_2025-11-19_14-30-00.mp4",
    "start_abs_time": "2025-11-19 14:30:00.123",
    "stop_abs_time": "2025-11-19 14:30:10.456",
    "start_timespec_sec": 1732024200,
    "start_timespec_nsec": 123456789,
    "stop_timespec_sec": 1732024210,
    "stop_timespec_nsec": 456789012,
    "recorded_frames": 300,
    "average_fps": 29.97,
    "start_frame": 100,
    "stop_frame": 400,
    ...
}
```

### Frame Extractor Output

The frame extractor creates a directory named `{video_basename}_frames/` containing:

- **PNG files**: `frame_000001.png`, `frame_000002.png`, etc. (one per frame)
- **index.json**: Metadata file with frame information

Example `index.json`:
```json
{
    "source_video": "Camera_1_2025-11-19_14-30-00.mp4",
    "source_metadata": "Camera_1_2025-11-19_14-30-00.mp4.json",
    "total_frames": 300,
    "fps": 29.97,
    "frames": [
        {
            "frame_number": 1,
            "filename": "frame_000001.png",
            "timestamp": "2025-11-19 14:30:00.123",
            "timespec_sec": 1732024200,
            "timespec_nsec": 123456789
        },
        {
            "frame_number": 2,
            "filename": "frame_000002.png",
            "timestamp": "2025-11-19 14:30:00.157",
            "timespec_sec": 1732024200,
            "timespec_nsec": 156790122
        },
        ...
    ]
}
```

## Project Structure

```
video-recorder/
├── CMakeLists.txt          # Root CMake configuration
├── config.json             # Configuration file
├── README.md               # This file
├── include/                # Header files
│   ├── gtk_stream_buf.h    # GTK stream buffer for log redirection
│   ├── video_controller.h  # Main controller logic
│   ├── video_pipeline.h    # GStreamer pipeline wrapper
│   └── video_preview.h     # GTK video window
└── src/                    # Source files
    ├── frame_extractor.cpp # Frame extraction tool
    ├── gtk_stream_buf.cpp
    ├── main.cpp            # Entry point
    ├── video_controller.cpp
    ├── video_pipeline.cpp
    └── video_preview.cpp
```

## License

MIT License
