# VR Renderer-Streamer

A real-time 3D renderer with fisheye projection that streams both RGB and depth data over UDP to [Unity VR](https://github.com/aharnishp/perspective-shift-correction-unity) application for perspective shift correction. This system improves user experience for VR perception of remote environment (where this script runs) by providing accurate 6DoF tracking data and synchronized visual feeds.

This is just a streamer for the receiver system, refer to the [Unity Project](https://github.com/aharnishp/perspective-shift-correction-unity). This script uses cornell box for testing, and currently fish eye projection.

Refer to previous commits for equirectangular projection code.

## Overview

This application renders a 3D scene using a custom fisheye projection and streams the resulting frames to a Unity VR client. The streamer receives 6DoF pose data from the VR headset and uses it to render the scene from the correct perspective, enabling seamless perspective shift correction in VR.

### Key Features

- **Real-time Fisheye Rendering**: Custom fisheye projection with adjustable field of view (60°-360°)
- **6DoF Pose Tracking**: Receives and processes position/rotation data from VR headsets
- **Dual Stream Output**: Synchronized RGB (JPEG) and depth data streaming
- **Network Streaming**: UDP-based streaming optimized for low latency
- **Manual Override**: Mouse/keyboard controls for debugging and manual camera control
- **Depth Visualization**: Real-time depth buffer visualization and inversion controls

## Architecture

```
Unity VR Client ←--→ VR Renderer-Streamer
     ↑                        ↓
   Pose Data              RGB + Depth
   (Port 8052)           (Port 8051)
```

### Data Flow

1. **Pose Reception**: VR headset pose data received on UDP port 8052
2. **Scene Rendering**: 3D scene rendered with fisheye projection from tracked pose
3. **Frame Streaming**: RGB (JPEG) and depth data streamed to Unity client on port 8051
4. **Perspective Correction**: Unity applies received data for immersive VR experience

## Network Protocol

### Pose Input (Port 8052)
Receives 28-byte UDP packets containing:
- Position: `float[3]` - X, Y, Z coordinates
- Rotation: `float[4]` - Quaternion (X, Y, Z, W)

### Stream Output (Port 8051)
Sends chunked UDP packets with:
- **Packet Header**: Frame ID, chunk index, total chunks, data type
- **Frame Metadata**: Timestamp, camera position, camera rotation
- **Payload**: JPEG image data or 8-bit depth data

## Configuration

### Network Settings
```cpp
const char* TARGET_IP = "192.168.31.144";  // Unity client IP
const unsigned short TARGET_PORT = 8051;    // Stream output port
const unsigned short POSE_LISTEN_PORT = 8052; // Pose input port
```

### Rendering Settings
```cpp
const int SCREEN_WIDTH = 1500;
const int SCREEN_HEIGHT = 1500;
const int JPEG_QUALITY = 75;
const int SEND_INTERVAL_MS = 33;  // ~30 FPS
```

### Depth Configuration
```cpp
const float DEPTH_NEAR = 0.1f;
const float DEPTH_FAR = 400.0f;
const float DEPTH_SCALE_8BIT = 128.0f;
```

## Controls

### Camera Controls
- **Mouse**: Look around (when in manual mode)
- **WASD**: Move camera
- **Space/Ctrl**: Move up/down

### System Controls
- **T**: Toggle between VR tracking and manual control
- **P**: Pause/resume streaming
- **V**: Toggle depth buffer visualization
- **I**: Invert depth map values
- **-/=**: Adjust fisheye field of view

### VR Tracking Adjustments
- **X**: Invert pitch (X-axis rotation)
- **Y**: Invert yaw (Y-axis rotation) 
- **Z**: Invert roll (Z-axis rotation)

## Build Requirements

### Dependencies
- **SDL2**: Window management and input handling
- **GLM**: Mathematics library for 3D transformations
- **STB Image Write**: JPEG encoding for stream output
- **POSIX Sockets**: Network communication (Unix/Linux/macOS)

### Platform Support
- Linux/macOS (primary)
- Windows (with socket API adaptations)

## Usage

### Basic Setup

1. **Configure Network**:
   ```cpp
   const char* TARGET_IP = "YOUR_UNITY_CLIENT_IP";
   ```

2. **Build and Run**:
   ```bash
   g++ -o vr-streamer main.cpp -lSDL2 -lGL
   ./vr-streamer
   ```

3. **Connect Unity Client**: Ensure Unity VR application is listening on configured ports

### VR Integration

The application automatically switches to tracking mode when pose data is received:

1. Unity VR client sends pose data to port 8052
2. Renderer applies pose with configurable axis inversions
3. Scene rendered from VR perspective with fisheye projection
4. RGB and depth streams sent to Unity for perspective correction

### Manual Testing

Use manual control mode for debugging:
- Press **T** to disable tracking
- Use mouse/keyboard for camera control
- Monitor console output for status messages

## Advanced Features

### Perspective Shift Correction

The fisheye rendering enables accurate perspective shift correction in VR by:
- Providing wide field-of-view imagery (up to 360°)
- Maintaining geometric accuracy across the entire view
- Synchronizing depth data for proper occlusion handling

### Adaptive Streaming

- Automatic frame rate adjustment based on network conditions
- Chunked packet transmission for large frames
- Metadata synchronization for temporal alignment

### Coordinate System Handling

The system handles coordinate system conversions between:
- Unity's left-handed coordinate system
- OpenGL's right-handed coordinate system
- VR tracking coordinate systems

## Performance Optimization

- Custom rasterization with fisheye projection
- Efficient depth buffer management
- UDP streaming with minimal latency
- Perspective-correct interpolation
- Near-plane clipping for stability

## Troubleshooting

### Common Issues

1. **No Pose Data Received**:
   - Check Unity client is sending to correct IP/port
   - Verify firewall settings allow UDP traffic

2. **Streaming Latency**:
   - Adjust `SEND_INTERVAL_MS` and `JPEG_QUALITY`
   - Check network bandwidth and stability

3. **Coordinate System Issues**:
   - Use X/Y/Z keys to adjust rotation inversions
   - Verify Unity and renderer coordinate system alignment

### Debug Output

The application provides detailed console logging:
- Pose reception status
- Streaming statistics
- Camera mode changes
- Network connection status

## License

[Specify your license here]

## Contributing

[Add contribution guidelines if applicable]
