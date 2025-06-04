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

## Depth Buffer Controls
- Press **I** to toggle depth map inversion ON/OFF
- When ON: Inverts depth values (far becomes near, near becomes far)
- Useful for different depth visualization requirements

## VR Rotation Inversion Controls (Global Settings)
- Press **X** to toggle pitch (X-axis) rotation inversion ON/OFF
- Press **Y** to toggle yaw (Y-axis) rotation inversion ON/OFF  
- Press **Z** to toggle roll (Z-axis) rotation inversion ON/OFF
- These settings apply globally when in VR tracking mode
- Useful if VR camera movement feels opposite to reality
- Each axis can be inverted independently

## VR Position Inversion Controls (Global Settings)
- Press **Q** to toggle X-axis position inversion ON/OFF
- Press **E** to toggle Y-axis position inversion ON/OFF
- Press **R** to toggle Z-axis position inversion ON/OFF
- These settings apply globally when in VR tracking mode
- Useful if VR camera position feels opposite to reality
- Each axis can be inverted independently

## Usage
1. Start VRStreamer: `./VRStreamer`
2. Start Unity VR scene
3. Press **T** to toggle VR tracking (ON by default)
4. Move your head in VR - camera should follow
5. Use mouse/keyboard to add manual adjustments on top of VR tracking
6. Press **I** to invert depth map if needed
7. Press **X**, **Y**, or **Z** to invert individual rotation axes if VR movement feels wrong
8. Press **Q**, **E**, or **R** to invert individual position axes if VR movement feels wrong
9. Press **T** again to return to manual-only mode

## Status Messages
- "[Camera] Tracking mode: ON/OFF" - VR tracking state
- "[Depth] Depth map inversion: ON/OFF" - Depth inversion state
- "[VR Rotation] Pitch/Yaw/Roll (X/Y/Z-axis) inversion: ON/OFF" - Rotation axis inversion states
- "[VR Position] X/Y/Z-axis inversion: ON/OFF" - Position axis inversion states
- "[Pose Receiver] Received pose" - VR data received successfully
