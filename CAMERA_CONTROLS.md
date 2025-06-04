# 6DoF Camera Control Instructions

## Camera Control Modes

### Manual Control Mode (Default)
- Mouse: Look around (yaw/pitch)
- WASD: Move forward/back/left/right
- Space: Move up
- Left Ctrl: Move down
- Arrow keys: Move light source
- Page Up/Down: Move light up/down

### VR Tracking Mode
- Press **T** to toggle VR tracking mode ON/OFF
- When ON: Camera follows received pose data from Unity VR
- When ON + manual input: Adds offset to VR pose
  - Mouse: Add rotation offset to VR head rotation
  - WASD/Space/Ctrl: Add position offset to VR head position
  - Offsets automatically reset after 2 seconds of no input

## Usage
1. Start VRStreamer: `./VRStreamer`
2. Start Unity VR scene
3. Press **T** to enable VR tracking
4. Move your head in VR - camera should follow
5. Use mouse/keyboard to add manual adjustments on top of VR tracking
6. Press **T** again to return to manual-only mode

## Status Messages
- "[Camera] Tracking mode: ON" - Now following VR pose
- "[Camera] Tracking mode: OFF" - Back to manual control
- "[Pose Receiver] Received pose" - VR data received successfully
