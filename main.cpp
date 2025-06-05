#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "ExtendedSDL2Aux.h"
#include "TestModel.h"
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <iomanip>

// Networking includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>

// STB Image Write include
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define fori(i,n) for(int i=0; i<n; ++i)

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::ivec2;

// Cross-platform socket definitions
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close

// === Streaming Structures ===
struct MemoryBuffer {
    unsigned char* data;
    size_t size;
    size_t capacity;

    MemoryBuffer() : data(nullptr), size(0), capacity(0) {}
    ~MemoryBuffer() {
        if (data) {
            free(data);
            data = nullptr;
        }
    }
    MemoryBuffer(const MemoryBuffer&) = delete;
    MemoryBuffer& operator=(const MemoryBuffer&) = delete;
};

#pragma pack(push, 1)
struct PacketHeader {
    uint32_t frameId;
    uint32_t chunkIndex;
    uint32_t totalChunks;
    uint8_t dataType; // 0 for JPEG, 1 for depth
};

struct FrameMetadata {
    uint64_t timestamp_us;  // Timestamp in microseconds since epoch
    float cam_pos_x;        // Camera position X (world coordinates)
    float cam_pos_y;        // Camera position Y (world coordinates) 
    float cam_pos_z;        // Camera position Z (world coordinates)
    float cam_rot_x;        // Camera rotation quaternion X (world coordinates)
    float cam_rot_y;        // Camera rotation quaternion Y (world coordinates)
    float cam_rot_z;        // Camera rotation quaternion Z (world coordinates)
    float cam_rot_w;        // Camera rotation quaternion W (world coordinates)
};
#pragma pack(pop)

// === Renderer Structures ===
struct Pixel {
    int x, y;
    float zinv;
    vec3 pos_world_times_zinv;
};

struct Vertex {
    vec3 position;
};

// === Configuration ===
const int SCREEN_WIDTH = 1000;
const int SCREEN_HEIGHT = 1000;
const char* TARGET_IP = "192.168.31.144";
// const char* TARGET_IP = "127.0.0.1";
const unsigned short TARGET_PORT = 8051;
const int JPEG_QUALITY = 75;
const int SEND_INTERVAL_MS = 33; // ~30 FPS
const int MAX_UDP_PACKET_SIZE = 1400;
const int HEADER_SIZE = sizeof(PacketHeader);
const int METADATA_SIZE = sizeof(FrameMetadata);
const int MAX_CHUNK_DATA_SIZE = MAX_UDP_PACKET_SIZE - HEADER_SIZE;
const int MAX_FIRST_CHUNK_DATA_SIZE = MAX_UDP_PACKET_SIZE - HEADER_SIZE - METADATA_SIZE;
const float MY_PI = 3.141592653f;

// Depth buffer configuration
const float DEPTH_NEAR = 0.1f;          // Near clipping plane
const float DEPTH_FAR = 100.0f;         // Far clipping plane
const float DEPTH_SCALE = 1000.0f;      // Convert to millimeters

// === Global Variables ===
ExtendedSDL2Aux *sdlAux;
int t;
vector<Triangle> triangles;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
std::atomic<bool> keep_running(true);

// Camera
vec3 cameraPos(0, 0, -3.001);
mat3 R;
float yaw = 0.0f;
float pitch = 0.0f;
float focal = (SCREEN_HEIGHT + SCREEN_WIDTH) * (0.1f);

// === 6DoF Camera Control System ===
// Manual control state
bool manual_control_active = false;
float manual_yaw = 0.0f;
float manual_pitch = 0.0f;
vec3 manual_position_offset(0.0f);
std::chrono::steady_clock::time_point last_mouse_movement;

// Base pose from VR tracking
vec3 base_position(0, 0, -3.001);
glm::quat base_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
bool use_tracking_pose = true;

// Store original world pose for metadata (not converted to our coordinate system)
vec3 world_position = vec3(0.0f, 0.0f, 3.0f);
glm::quat world_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
bool has_world_pose = false;

// Depth buffer control
bool invert_depth_map = false;

// VR rotation inversion toggles (global settings for tracked mode)
bool invert_pitch = true;  // X-axis rotation
bool invert_yaw = false;    // Y-axis rotation  
bool invert_roll = false;   // Z-axis rotation

// VR translation inversion toggles (global settings for tracked mode)
bool invert_pos_x = false;  // X-axis translation
bool invert_pos_y = true;  // Y-axis translation
bool invert_pos_z = true;  // Z-axis translation

// Lighting
vec3 lightPos(0, -0.5, -0.7);
vec3 lightPower = 3.0f * vec3(1, 1, 1);
vec3 indirectLightPowerPerArea = 0.5f * vec3(1, 1, 1);

// Current triangle properties
vec3 currentColor;
vec3 currentTriangleNormal_world;

// Streaming variables
SOCKET image_sender_socket;
sockaddr_in receiver_addr;
uint32_t frame_count = 0;

// === Pose Receiving Structures and Variables ===
struct Pose6DoF {
    glm::vec3 position = glm::vec3(0.0f, 0.0f, 3.0f);
    glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // w, x, y, z
};

// Pose receiving configuration
const unsigned short POSE_LISTEN_PORT = 8052;

// Shared data for pose receiving
Pose6DoF shared_pose;
std::mutex pose_mutex;
std::atomic<bool> pose_received(false);

// Pose receiving variables
SOCKET pose_receiver_socket;
std::thread pose_receiver_thread;

// === Pose Receiving Thread Function ===
void pose_receiver_thread_func() {
    std::cout << "[Pose Receiver] Thread starting..." << std::endl;
    
    // Create UDP socket for receiving pose data
    pose_receiver_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (pose_receiver_socket == INVALID_SOCKET) {
        std::cerr << "[Pose Receiver] Failed to create socket" << std::endl;
        return;
    }
    
    // Set up address for listening
    sockaddr_in listen_addr;
    memset(&listen_addr, 0, sizeof(listen_addr));
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = INADDR_ANY;
    listen_addr.sin_port = htons(POSE_LISTEN_PORT);
    
    // Bind socket to listen address
    if (::bind(pose_receiver_socket, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) == SOCKET_ERROR) {
        std::cerr << "[Pose Receiver] Failed to bind to port " << POSE_LISTEN_PORT << std::endl;
        closesocket(pose_receiver_socket);
        return;
    }
    
    std::cout << "[Pose Receiver] Listening on port " << POSE_LISTEN_PORT << std::endl;
    
    // Set up for non-blocking receive with timeout
    pollfd poll_fd;
    poll_fd.fd = pose_receiver_socket;
    poll_fd.events = POLLIN;
    
    const int TIMEOUT_MS = 1000; // 1 second timeout
    char buffer[28]; // Pose data is 28 bytes: 3 floats for position + 4 floats for quaternion
    
    while (keep_running) {
        // Check for incoming data with timeout
        int poll_result = poll(&poll_fd, 1, TIMEOUT_MS);
        
        if (poll_result > 0 && (poll_fd.revents & POLLIN)) {
            sockaddr_in sender_addr;
            socklen_t sender_len = sizeof(sender_addr);
            
            // Receive pose data
            ssize_t bytes_received = recvfrom(pose_receiver_socket, buffer, sizeof(buffer), 0, 
                                              (struct sockaddr*)&sender_addr, &sender_len);
            
            if (bytes_received == 28) { // Expected size for pose data
                // Parse pose data (little-endian floats)
                float* float_data = reinterpret_cast<float*>(buffer);
                
                Pose6DoF new_pose;
                new_pose.position.x = float_data[0]; // Unity world X
                new_pose.position.y = float_data[1]; // Unity world Y  
                new_pose.position.z = float_data[2]; // Unity world Z
                new_pose.rotation.x = float_data[3]; // Quaternion X (world space)
                new_pose.rotation.y = float_data[4]; // Quaternion Y (world space)
                new_pose.rotation.z = float_data[5]; // Quaternion Z (world space)
                new_pose.rotation.w = float_data[6]; // Quaternion W (world space)
                
                // Update shared pose data thread-safely
                {
                    std::lock_guard<std::mutex> lock(pose_mutex);
                    shared_pose = new_pose;
                    pose_received = true;
                    
                    // Store original world pose for metadata (exactly as received from Unity)
                    world_position = new_pose.position;
                    world_rotation = new_pose.rotation;
                    has_world_pose = true;
                }
                
                // Log received pose
                std::cout << "[Pose Receiver] Received pose - Position: ("
                          << std::fixed << std::setprecision(3)
                          << new_pose.position.x << ", "
                          << new_pose.position.y << ", "
                          << new_pose.position.z << ") Rotation: ("
                          << new_pose.rotation.x << ", "
                          << new_pose.rotation.y << ", "
                          << new_pose.rotation.z << ", "
                          << new_pose.rotation.w << ")" << std::endl;
                          
            } else if (bytes_received > 0) {
                std::cerr << "[Pose Receiver] Received unexpected data size: " << bytes_received << " bytes (expected 28)" << std::endl;
                
                // Debug: Print the received bytes in hex format
                std::cout << "[Pose Receiver Debug] Received bytes: ";
                for (ssize_t i = 0; i < std::min(bytes_received, static_cast<ssize_t>(16)); i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (unsigned char)buffer[i] << " ";
                }
                std::cout << std::dec << std::endl;
            }
        } else if (poll_result == 0) {
            // Timeout - continue loop
            continue;
        } else {
            // Error
            if (keep_running) {
                std::cerr << "[Pose Receiver] Poll error" << std::endl;
            }
            break;
        }
    }
    
    std::cout << "[Pose Receiver] Thread shutting down..." << std::endl;
    closesocket(pose_receiver_socket);
}

// === Streaming Functions ===
void write_memory_buffer(void* context, void* data, int size) {
    if (!context || !data || size <= 0) return;

    MemoryBuffer* buffer = static_cast<MemoryBuffer*>(context);
    size_t new_size = buffer->size + size;

    if (new_size > buffer->capacity) {
        size_t new_capacity = (buffer->capacity == 0) ?
            std::max(static_cast<size_t>(size * 2), static_cast<size_t>(4096)) :
            std::max(buffer->capacity * 2, new_size);

        unsigned char* new_data = static_cast<unsigned char*>(
            buffer->data ? realloc(buffer->data, new_capacity) :
                         malloc(new_capacity));

        if (!new_data) {
            return;
        }
        buffer->data = new_data;
        buffer->capacity = new_capacity;
    }

    memcpy(buffer->data + buffer->size, data, size);
    buffer->size = new_size;
}

void extractRGBFromSDL(ExtendedSDL2Aux* sdlAux, std::vector<unsigned char>& rgb_buffer) {
    sdlAux->extractRGBBuffer(rgb_buffer);
}

void convertDepthBufferTo16Bit(std::vector<uint16_t>& depth_16bit) {
    depth_16bit.resize(SCREEN_WIDTH * SCREEN_HEIGHT);
    
    for (int y = 0; y < SCREEN_HEIGHT; ++y) {
        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            int idx = y * SCREEN_WIDTH + x;
            // Invert Y coordinate to match color buffer orientation
            int inverted_y = SCREEN_HEIGHT - 1 - y;
            float zinv = depthBuffer[inverted_y][x];
            
            if (zinv <= 0.0f) {
                // No depth or behind camera - set to 0 (error/infinite)
                depth_16bit[idx] = 0;
            } else {
                // Convert from 1/z to actual depth distance
                float depth_world = 1.0f / zinv;
                
                // Convert to millimeters and clamp to 16-bit range
                uint32_t depth_mm = static_cast<uint32_t>(depth_world * DEPTH_SCALE);
                uint16_t depth_value = static_cast<uint16_t>(std::min(depth_mm, static_cast<uint32_t>(65535)));
                
                // Apply depth inversion if enabled
                if (invert_depth_map) {
                    depth_16bit[idx] = 65535 - depth_value;
                } else {
                    depth_16bit[idx] = depth_value;
                }
            }
        }
    }
}

bool initializeStreaming() {
    image_sender_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (image_sender_socket == INVALID_SOCKET) {
        perror("Failed to create socket");
        return false;
    }

    memset(&receiver_addr, 0, sizeof(receiver_addr));
    receiver_addr.sin_family = AF_INET;
    receiver_addr.sin_port = htons(TARGET_PORT);
    if (inet_pton(AF_INET, TARGET_IP, &receiver_addr.sin_addr) <= 0) {
        std::cerr << "Invalid target IP address: " << TARGET_IP << std::endl;
        closesocket(image_sender_socket);
        return false;
    }

    std::cout << "[Streamer] Ready to send to " << TARGET_IP << ":" << TARGET_PORT << std::endl;
    return true;
}

// Helper function to get current frame metadata (timestamp and camera pose)
FrameMetadata getCurrentFrameMetadata() {
    FrameMetadata metadata;
    
    // Get current timestamp in microseconds since epoch
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    metadata.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    
    // Use the original world pose if available (maintains world coordinates throughout pipeline)
    if (has_world_pose) {
        // Send back the original world pose exactly as received from Unity
        // This maintains consistency in the coordinate reference frame
        metadata.cam_pos_x = world_position.x;
        metadata.cam_pos_y = world_position.y;
        metadata.cam_pos_z = world_position.z;  // Keep original world coordinates
        
        // Send back the original world rotation exactly as received from Unity
        metadata.cam_rot_x = world_rotation.x;
        metadata.cam_rot_y = world_rotation.y;
        metadata.cam_rot_z = world_rotation.z;  // Keep original world coordinates
        metadata.cam_rot_w = world_rotation.w;
    } else {
        // Fallback to current camera pose (convert from our coordinate system to Unity world coordinates)
        // Our system: Y-up, Z-backward, X-right (right-handed)
        // Unity world: Y-up, Z-forward, X-right (left-handed)
        // 
        // IMPORTANT: Send the actual camera pose in Unity world coordinates, 
        // NOT the "un-inverted" version. The inversions are user preferences
        // for input handling, but the output pose should match what was rendered.
        metadata.cam_pos_x = cameraPos.x;
        metadata.cam_pos_y = cameraPos.y;
        metadata.cam_pos_z = -cameraPos.z;  // Convert Z from our backward system to Unity's forward system
        
        // Convert rotation matrix back to quaternion and then to Unity coordinates
        glm::quat cam_quat = glm::quat_cast(R);
        
        // Convert quaternion from our coordinate system to Unity's
        // Our system: Y-up, Z-backward, X-right (right-handed)
        // Unity world: Y-up, Z-forward, X-right (left-handed)
        metadata.cam_rot_x = cam_quat.x;
        metadata.cam_rot_y = cam_quat.y;
        metadata.cam_rot_z = -cam_quat.z;  // Invert Z component for coordinate system conversion
        metadata.cam_rot_w = cam_quat.w;
    }
    
    return metadata;
}

bool sendDataPackets(const unsigned char* data, size_t data_size, uint8_t data_type) {
    // Calculate total chunks considering metadata in first chunk
    size_t first_chunk_data_size = std::min(data_size, static_cast<size_t>(MAX_FIRST_CHUNK_DATA_SIZE));
    size_t remaining_data_size = data_size - first_chunk_data_size;
    uint32_t total_chunks = 1; // At least one chunk for first chunk with metadata
    if (remaining_data_size > 0) {
        total_chunks += (remaining_data_size + MAX_CHUNK_DATA_SIZE - 1) / MAX_CHUNK_DATA_SIZE;
    }
    
    // Get frame metadata for first packet
    FrameMetadata metadata = getCurrentFrameMetadata();

    std::vector<unsigned char> packet_buffer(MAX_UDP_PACKET_SIZE);
    PacketHeader* header = reinterpret_cast<PacketHeader*>(packet_buffer.data());
    
    header->frameId = htonl(frame_count);
    header->totalChunks = htonl(total_chunks);
    header->dataType = data_type; // 0 for JPEG, 1 for depth

    size_t bytes_remaining = data_size;
    const unsigned char* data_ptr = data;

    for (uint32_t chunk_idx = 0; chunk_idx < total_chunks; chunk_idx++) {
        header->chunkIndex = htonl(chunk_idx);

        int packet_size = HEADER_SIZE;
        
        if (chunk_idx == 0) {
            // First chunk: include metadata
            memcpy(packet_buffer.data() + HEADER_SIZE, &metadata, METADATA_SIZE);
            packet_size += METADATA_SIZE;
            
            // Add first chunk of data after metadata
            if (bytes_remaining > 0) {
                int current_chunk_data_size = std::min(static_cast<size_t>(MAX_FIRST_CHUNK_DATA_SIZE), bytes_remaining);
                memcpy(packet_buffer.data() + HEADER_SIZE + METADATA_SIZE, data_ptr, current_chunk_data_size);
                packet_size += current_chunk_data_size;
                data_ptr += current_chunk_data_size;
                bytes_remaining -= current_chunk_data_size;
            }
        } else {
            // Subsequent chunks: data only
            int current_chunk_data_size = std::min(static_cast<size_t>(MAX_CHUNK_DATA_SIZE), bytes_remaining);
            memcpy(packet_buffer.data() + HEADER_SIZE, data_ptr, current_chunk_data_size);
            packet_size += current_chunk_data_size;
            data_ptr += current_chunk_data_size;
            bytes_remaining -= current_chunk_data_size;
        }

        int bytes_sent = sendto(image_sender_socket,
            packet_buffer.data(),
            packet_size,
            0,
            reinterpret_cast<const struct sockaddr*>(&receiver_addr),
            sizeof(receiver_addr));

        if (bytes_sent == SOCKET_ERROR) {
            perror("sendto failed");
            return false;
        }

        if (total_chunks > 1 && chunk_idx < total_chunks - 1) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
    return true;
}

void streamFrame() {
    frame_count++;
    
    // Extract RGB data from SDL buffer
    std::vector<unsigned char> rgb_buffer;
    extractRGBFromSDL(sdlAux, rgb_buffer);
    
    // Encode as JPEG
    MemoryBuffer jpeg_buffer;
    int jpeg_success = stbi_write_jpg_to_func(
        write_memory_buffer,
        &jpeg_buffer,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        3, // RGB channels
        rgb_buffer.data(),
        JPEG_QUALITY
    );

    // Send color JPEG data
    if (jpeg_success && jpeg_buffer.data && jpeg_buffer.size > 0) {
        sendDataPackets(jpeg_buffer.data, jpeg_buffer.size, 0); // 0 = JPEG color
    }

    // Convert and send depth data
    std::vector<uint16_t> depth_16bit;
    convertDepthBufferTo16Bit(depth_16bit);
    
    // Send depth data as raw binary (16-bit values)
    if (!depth_16bit.empty()) {
        const unsigned char* depth_data = reinterpret_cast<const unsigned char*>(depth_16bit.data());
        size_t depth_size = depth_16bit.size() * sizeof(uint16_t);
        sendDataPackets(depth_data, depth_size, 1); // 1 = depth data
    }
}

// === 6DoF Camera Helper Functions ===
mat3 quaternionToRotationMatrix(const glm::quat& q) {
    // Convert quaternion to rotation matrix
    // Unity uses left-handed coordinates, we may need to adjust
    float x = q.x, y = q.y, z = q.z, w = q.w;
    
    mat3 result;
    result[0][0] = 1.0f - 2.0f * (y*y + z*z);
    result[0][1] = 2.0f * (x*y - w*z);
    result[0][2] = 2.0f * (x*z + w*y);
    
    result[1][0] = 2.0f * (x*y + w*z);
    result[1][1] = 1.0f - 2.0f * (x*x + z*z);
    result[1][2] = 2.0f * (y*z - w*x);
    
    result[2][0] = 2.0f * (x*z - w*y);
    result[2][1] = 2.0f * (y*z + w*x);
    result[2][2] = 1.0f - 2.0f * (x*x + y*y);
    
    return result;
}

mat3 createRotationMatrix(float yaw, float pitch) {
    mat3 R_yaw = glm::mat3(
        glm::vec3(cos(yaw), 0.0f, sin(yaw)),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(-sin(yaw), 0.0f, cos(yaw))
    );

    mat3 R_pitch = glm::mat3(
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, cos(pitch), -sin(pitch)),
        glm::vec3(0.0f, sin(pitch), cos(pitch))
    );
    
    return R_pitch * R_yaw;
}

// Helper function to apply rotation axis inversions to a quaternion
glm::quat applyRotationInversions(const glm::quat& quat) {
    glm::quat result = quat;
    
    // Apply inversions by negating specific quaternion components
    // This approach avoids Euler angle conversion and works directly with quaternions
    if (invert_pitch) {
        // Invert pitch by negating X component (around X-axis)
        result.x = -result.x;
    }
    if (invert_yaw) {
        // Invert yaw by negating Y component (around Y-axis)
        result.y = -result.y;
    }
    if (invert_roll) {
        // Invert roll by negating Z component (around Z-axis)
        result.z = -result.z;
    }
    
    return result;
}

// Helper function to apply translation axis inversions to a position vector
vec3 applyPositionInversions(const vec3& position) {
    vec3 result = position;
    
    // Apply inversions by negating specific position components
    if (invert_pos_x) {
        // Invert X-axis translation
        result.x = -result.x;
    }
    if (invert_pos_y) {
        // Invert Y-axis translation
        result.y = -result.y;
    }
    if (invert_pos_z) {
        // Invert Z-axis translation
        result.z = -result.z;
    }
    
    return result;
}

void updateCameraPose() {
    if (use_tracking_pose && pose_received.load()) {
        // Get latest pose data
        Pose6DoF current_pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            current_pose = shared_pose;
        }
        
        // Convert Unity world coordinates to our coordinate system
        // Unity world: Y-up, Z-forward, X-right (left-handed)
        // Our system: Y-up, Z-backward, X-right (right-handed)
        vec3 unity_position = vec3(current_pose.position.x, current_pose.position.y, -current_pose.position.z);
        
        // Apply translation axis inversions if enabled
        base_position = applyPositionInversions(unity_position);
        
        // Convert quaternion to rotation matrix
        // Note: May need to adjust quaternion components for coordinate system differences
        glm::quat adjusted_quat = glm::quat(current_pose.rotation.w, current_pose.rotation.x, current_pose.rotation.y, -current_pose.rotation.z);
        
        // Apply rotation axis inversions if enabled
        glm::quat inverted_quat = applyRotationInversions(adjusted_quat);
        
        mat3 base_rotation_matrix = quaternionToRotationMatrix(inverted_quat);
        
        // Apply manual control offset if active
        if (manual_control_active) {
            // Create manual rotation matrix
            mat3 manual_rotation = createRotationMatrix(manual_yaw, manual_pitch);
            
            // Combine rotations: apply manual rotation after base rotation
            R = manual_rotation * base_rotation_matrix;
            
            // Apply manual position offset in world space
            cameraPos = base_position + manual_position_offset;
        } else {
            // Use pure tracking data
            R = base_rotation_matrix;
            cameraPos = base_position;
        }
    } else {
        // Fallback to manual control only (original behavior)
        R = createRotationMatrix(yaw, pitch);
        // cameraPos updated in Update() function
    }
}

// === Renderer Functions ===
void Update(void);
void Draw(void);
void VertexShaderPixel(const Vertex& v_in, Pixel& p_out);
void InterpolatePixel(Pixel a, Pixel b, vector<Pixel>& result);
void ComputePolygonRowsPixel(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels, int& minHeight, int& maxHeight);
void DrawLineHorizontalDepth(const Pixel& start, const Pixel& end);
void DrawRowsPixel(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, int polygonMinY, int polygonMaxY);
void DrawPolygonPixel(const vector<Vertex>& vertices);
void PixelShader(const Pixel& p);

void Update(void) {
    int t2 = SDL_GetTicks();
    float dt = float(t2 - t);
    t = t2;

    int dx, dy;
    SDL_GetRelativeMouseState(&dx, &dy);
    
    const Uint8* keystate = SDL_GetKeyboardState(NULL);
    
    // Check if user is providing manual input
    bool mouse_input = (dx != 0 || dy != 0);
    bool keyboard_input = (keystate[SDL_SCANCODE_W] || keystate[SDL_SCANCODE_S] || 
                          keystate[SDL_SCANCODE_A] || keystate[SDL_SCANCODE_D] || 
                          keystate[SDL_SCANCODE_SPACE] || keystate[SDL_SCANCODE_LCTRL]);
    
    // Toggle tracking mode with T key
    static bool t_key_was_pressed = false;
    if (keystate[SDL_SCANCODE_T] && !t_key_was_pressed) {
        use_tracking_pose = !use_tracking_pose;
        std::cout << "[Camera] Tracking mode: " << (use_tracking_pose ? "ON" : "OFF") << std::endl;
        if (!use_tracking_pose) {
            // Reset manual offsets when disabling tracking
            manual_yaw = yaw;
            manual_pitch = pitch;
            manual_position_offset = vec3(0.0f);
        }
    }
    t_key_was_pressed = keystate[SDL_SCANCODE_T];
    
    // Toggle depth map inversion with I key
    static bool i_key_was_pressed = false;
    if (keystate[SDL_SCANCODE_I] && !i_key_was_pressed) {
        invert_depth_map = !invert_depth_map;
        std::cout << "[Depth] Depth map inversion: " << (invert_depth_map ? "ON" : "OFF") << std::endl;
    }
    i_key_was_pressed = keystate[SDL_SCANCODE_I];
    
    // Toggle VR rotation inversion with X, Y, Z keys (for pitch, yaw, roll)
    static bool x_key_was_pressed = false;
    if (keystate[SDL_SCANCODE_X] && !x_key_was_pressed) {
        invert_pitch = !invert_pitch;
        std::cout << "[VR Rotation] Pitch (X-axis) inversion: " << (invert_pitch ? "ON" : "OFF") << std::endl;
    }
    x_key_was_pressed = keystate[SDL_SCANCODE_X];
    
    static bool y_key_was_pressed = false;
    if (keystate[SDL_SCANCODE_Y] && !y_key_was_pressed) {
        invert_yaw = !invert_yaw;
        std::cout << "[VR Rotation] Yaw (Y-axis) inversion: " << (invert_yaw ? "ON" : "OFF") << std::endl;
    }
    y_key_was_pressed = keystate[SDL_SCANCODE_Y];
    
    static bool z_key_was_pressed = false;
    if (keystate[SDL_SCANCODE_Z] && !z_key_was_pressed) {
        invert_roll = !invert_roll;
        std::cout << "[VR Rotation] Roll (Z-axis) inversion: " << (invert_roll ? "ON" : "OFF") << std::endl;
    }
    z_key_was_pressed = keystate[SDL_SCANCODE_Z];
    
    // Toggle VR position inversion with Q, E, R keys (for X, Y, Z position)
    static bool q_key_was_pressed = false;
    if (keystate[SDL_SCANCODE_Q] && !q_key_was_pressed) {
        invert_pos_x = !invert_pos_x;
        std::cout << "[VR Position] X-axis inversion: " << (invert_pos_x ? "ON" : "OFF") << std::endl;
    }
    q_key_was_pressed = keystate[SDL_SCANCODE_Q];
    
    static bool e_key_was_pressed = false;
    if (keystate[SDL_SCANCODE_E] && !e_key_was_pressed) {
        invert_pos_y = !invert_pos_y;
        std::cout << "[VR Position] Y-axis inversion: " << (invert_pos_y ? "ON" : "OFF") << std::endl;
    }
    e_key_was_pressed = keystate[SDL_SCANCODE_E];
    
    static bool r_key_was_pressed = false;
    if (keystate[SDL_SCANCODE_R] && !r_key_was_pressed) {
        invert_pos_z = !invert_pos_z;
        std::cout << "[VR Position] Z-axis inversion: " << (invert_pos_z ? "ON" : "OFF") << std::endl;
    }
    r_key_was_pressed = keystate[SDL_SCANCODE_R];
    
    // Handle input based on mode
    if (use_tracking_pose) {
        // When using tracking, manual input creates offsets
        if (mouse_input || keyboard_input) {
            manual_control_active = true;
            last_mouse_movement = std::chrono::steady_clock::now();
            
            // Apply mouse input to manual rotation offset
            if (mouse_input) {
                if (dx != 0) {
                    manual_yaw -= dx * -0.005f;
                }
                if (dy != 0) {
                    manual_pitch -= dy * 0.005f;
                    const float pitchLimit = MY_PI / 2.0f - 0.01f;
                    manual_pitch = glm::clamp(manual_pitch, -pitchLimit, pitchLimit);
                }
            }
            
            // Apply keyboard input to manual position offset
            float moveSpeed = 0.005f * dt;
            
            // Calculate movement directions based on current combined rotation
            mat3 current_rotation = R; // Use current camera rotation
            mat3 cameraToWorld = glm::transpose(current_rotation);
            vec3 forward_world = cameraToWorld * vec3(0,0,-1);
            vec3 right_world = cameraToWorld * vec3(1,0,0);
            
            if (keystate[SDL_SCANCODE_W]) manual_position_offset -= forward_world * moveSpeed;
            if (keystate[SDL_SCANCODE_S]) manual_position_offset += forward_world * moveSpeed;
            if (keystate[SDL_SCANCODE_A]) manual_position_offset -= right_world * moveSpeed;
            if (keystate[SDL_SCANCODE_D]) manual_position_offset += right_world * moveSpeed;
            if (keystate[SDL_SCANCODE_SPACE]) manual_position_offset.y += moveSpeed;
            if (keystate[SDL_SCANCODE_LCTRL]) manual_position_offset.y -= moveSpeed;
        } else {
            // Check if enough time has passed to disable manual control
            auto now = std::chrono::steady_clock::now();
            auto time_since_input = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_mouse_movement).count();
            if (time_since_input > 2000) { // 2 seconds timeout
                manual_control_active = false;
                manual_yaw = 0.0f;
                manual_pitch = 0.0f;
                manual_position_offset = vec3(0.0f);
            }
        }
    } else {
        // Original manual control mode
        manual_control_active = false;
        
        if (mouse_input) {
            if (dx != 0) {
                yaw -= dx * -0.005f;
            }
            if (dy != 0) {
                pitch -= dy * 0.005f;
                const float pitchLimit = MY_PI / 2.0f - 0.01f;
                pitch = glm::clamp(pitch, -pitchLimit, pitchLimit);
            }
        }
        
        float moveSpeed = 0.005f * dt;
        mat3 cameraToWorld = glm::transpose(R);
        vec3 forward_world = cameraToWorld * vec3(0,0,-1);
        vec3 right_world = cameraToWorld * vec3(1,0,0);

        if (keystate[SDL_SCANCODE_W]) cameraPos -= forward_world * moveSpeed;
        if (keystate[SDL_SCANCODE_S]) cameraPos += forward_world * moveSpeed;
        if (keystate[SDL_SCANCODE_A]) cameraPos -= right_world * moveSpeed;
        if (keystate[SDL_SCANCODE_D]) cameraPos += right_world * moveSpeed;
        if (keystate[SDL_SCANCODE_SPACE]) cameraPos.y += moveSpeed;
        if (keystate[SDL_SCANCODE_LCTRL]) cameraPos.y -= moveSpeed;
    }

    // Light control (unchanged)
    float lightMoveSpeed = 0.005f * dt;
    if (keystate[SDL_SCANCODE_UP]) lightPos.z += lightMoveSpeed;
    if (keystate[SDL_SCANCODE_DOWN]) lightPos.z -= lightMoveSpeed;
    if (keystate[SDL_SCANCODE_LEFT]) lightPos.x -= lightMoveSpeed;
    if (keystate[SDL_SCANCODE_RIGHT]) lightPos.x += lightMoveSpeed;
    if (keystate[SDL_SCANCODE_PAGEUP]) lightPos.y += lightMoveSpeed;
    if (keystate[SDL_SCANCODE_PAGEDOWN]) lightPos.y -= lightMoveSpeed;

    // Update camera pose based on current mode
    updateCameraPose();
}

void VertexShaderPixel(const Vertex& v_in, Pixel& p_out) {
    vec3 position_camera_space = R * (v_in.position - cameraPos);

    if (position_camera_space.z < 0.001f) {
        p_out.x = -1;
        p_out.y = -1;
        p_out.zinv = 0;
        p_out.pos_world_times_zinv = vec3(0.0f);
        return;
    }

    p_out.zinv = 1.0f / position_camera_space.z;
    p_out.x = static_cast<int>(round(focal * position_camera_space.x * p_out.zinv + SCREEN_WIDTH / 2.0f));
    p_out.y = static_cast<int>(round(focal * position_camera_space.y * p_out.zinv + SCREEN_HEIGHT / 2.0f));
    p_out.pos_world_times_zinv = v_in.position * p_out.zinv;
}

void InterpolatePixel(Pixel a, Pixel b, vector<Pixel>& result) {
    int N = result.size();
    if (N == 0) return;
    if (N == 1) {
        result[0] = a;
        return;
    }

    float num_steps = static_cast<float>(N - 1);
    if (num_steps < 1e-5) {
        fori(i,N) result[i] = a;
        return;
    }

    float x_step = static_cast<float>(b.x - a.x) / num_steps;
    float y_step = static_cast<float>(b.y - a.y) / num_steps;
    float zinv_step = (b.zinv - a.zinv) / num_steps;
    vec3 pos_world_times_zinv_step = (b.pos_world_times_zinv - a.pos_world_times_zinv) / num_steps;

    float current_x = static_cast<float>(a.x);
    float current_y = static_cast<float>(a.y);
    float current_zinv = a.zinv;
    vec3 current_pos_world_times_zinv = a.pos_world_times_zinv;

    fori(i, N) {
        result[i].x = static_cast<int>(round(current_x));
        result[i].y = static_cast<int>(round(current_y));
        result[i].zinv = current_zinv;
        result[i].pos_world_times_zinv = current_pos_world_times_zinv;
        
        current_x += x_step;
        current_y += y_step;
        current_zinv += zinv_step;
        current_pos_world_times_zinv += pos_world_times_zinv_step;
    }
}

void ComputePolygonRowsPixel(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels, int& polygonMinY, int& polygonMaxY) {
    polygonMinY = SCREEN_HEIGHT;
    polygonMaxY = 0;

    int validVertices = 0;
    for (const auto& p : vertexPixels) {
        if (p.x == -1 && p.y == -1) continue;
        polygonMinY = std::min(polygonMinY, p.y);
        polygonMaxY = std::max(polygonMaxY, p.y);
        validVertices++;
    }
    if (validVertices < 3) {
        polygonMinY = SCREEN_HEIGHT;
        polygonMaxY = 0;
        return;
    }

    polygonMinY = std::max(0, polygonMinY);
    polygonMaxY = std::min(SCREEN_HEIGHT - 1, polygonMaxY);

    if (polygonMinY > polygonMaxY) return;

    for (int y = polygonMinY; y <= polygonMaxY; ++y) {
        leftPixels[y].x = SCREEN_WIDTH;
        leftPixels[y].y = y;
        leftPixels[y].zinv = 0.f;
        leftPixels[y].pos_world_times_zinv = vec3(0.f);

        rightPixels[y].x = 0;
        rightPixels[y].y = y;
        rightPixels[y].zinv = 0.f;
        rightPixels[y].pos_world_times_zinv = vec3(0.f);
    }

    for (size_t i = 0; i < vertexPixels.size(); ++i) {
        Pixel p1_orig = vertexPixels[i];
        Pixel p2_orig = vertexPixels[(i + 1) % vertexPixels.size()];

        if ((p1_orig.x == -1 && p1_orig.y == -1) || (p2_orig.x == -1 && p2_orig.y == -1)) continue;

        Pixel p1 = p1_orig;
        Pixel p2 = p2_orig;

        if (p1.y > p2.y) {
            std::swap(p1, p2);
        }

        if (p1.y == p2.y) continue;

        int startY_edge = std::max(p1.y, polygonMinY);
        int endY_edge = std::min(p2.y, polygonMaxY);

        if (startY_edge > endY_edge) continue;

        int dy_edge_full = p2.y - p1.y;
        if (dy_edge_full == 0) continue;

        vector<Pixel> edgeScanlinePixels(dy_edge_full + 1);
        InterpolatePixel(p1, p2, edgeScanlinePixels);

        for(int current_y_on_edge = 0; current_y_on_edge < edgeScanlinePixels.size(); ++current_y_on_edge) {
            const auto& edgePixel = edgeScanlinePixels[current_y_on_edge];
            int y = edgePixel.y;

            if (y >= polygonMinY && y <= polygonMaxY) {
                if (edgePixel.x < leftPixels[y].x) {
                    leftPixels[y] = edgePixel;
                }
                if (edgePixel.x > rightPixels[y].x) {
                    rightPixels[y] = edgePixel;
                }
            }
        }
    }
}

void DrawLineHorizontalDepth(const Pixel& startNode, const Pixel& endNode) {
    int y = startNode.y;

    if (y < 0 || y >= SCREEN_HEIGHT) {
        return;
    }

    int x_start = startNode.x;
    int x_end = endNode.x;

    int x_start_clamped = std::max(0, x_start);
    int x_end_clamped = std::min(SCREEN_WIDTH - 1, x_end);
    
    if (x_start_clamped > x_end_clamped) return;

    int dx_row = x_end - x_start;

    for (int x = x_start_clamped; x <= x_end_clamped; ++x) {
        Pixel currentPixel;
        currentPixel.x = x;
        currentPixel.y = y;

        if (abs(dx_row) < 1e-5) {
            currentPixel.zinv = startNode.zinv;
            currentPixel.pos_world_times_zinv = startNode.pos_world_times_zinv;
        } else {
            float alpha = static_cast<float>(x - x_start) / static_cast<float>(dx_row);
            alpha = glm::clamp(alpha, 0.0f, 1.0f);
            currentPixel.zinv = glm::mix(startNode.zinv, endNode.zinv, alpha);
            currentPixel.pos_world_times_zinv = glm::mix(startNode.pos_world_times_zinv, endNode.pos_world_times_zinv, alpha);
        }
        PixelShader(currentPixel);
    }
}

void DrawRowsPixel(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, int polygonMinY, int polygonMaxY) {
    for (int y = polygonMinY; y <= polygonMaxY; ++y) {
        if (y < 0 || y >= SCREEN_HEIGHT) continue;

        if (leftPixels[y].x <= rightPixels[y].x) {
            DrawLineHorizontalDepth(leftPixels[y], rightPixels[y]);
        }
    }
}

void DrawPolygonPixel(const vector<Vertex>& vertices) {
    int V = vertices.size();
    if (V < 3) return;

    vector<Pixel> vertexPixels(V);
    bool any_valid_vertex = false;
    fori(i, V) {
        VertexShaderPixel(vertices[i], vertexPixels[i]);
        if(vertexPixels[i].x != -1) any_valid_vertex = true;
    }

    if(!any_valid_vertex) return;

    vector<Pixel> leftPixels(SCREEN_HEIGHT);
    vector<Pixel> rightPixels(SCREEN_HEIGHT);
    
    int polygonMinY, polygonMaxY;

    ComputePolygonRowsPixel(vertexPixels, leftPixels, rightPixels, polygonMinY, polygonMaxY);
    
    if (polygonMinY <= polygonMaxY) {
       DrawRowsPixel(leftPixels, rightPixels, polygonMinY, polygonMaxY);
    }
}

void PixelShader(const Pixel& p) {
    int x = p.x;
    int y = p.y;

    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) {
        return;
    }

    if (p.zinv > depthBuffer[y][x]) {
        depthBuffer[y][x] = p.zinv;
        
        vec3 actual_pixel_world_pos;
        if (abs(p.zinv) < 1e-9) {
             sdlAux->putPixelWithCapture(x, y, vec3(0.1,0.1,0.1));
             return;
        } else {
            actual_pixel_world_pos = p.pos_world_times_zinv / p.zinv;
        }

        vec3 s = lightPos - actual_pixel_world_pos;
        float dist_sq = glm::dot(s, s);

        vec3 illumination_direct(0.0f);

        if (dist_sq > 1e-5f) {
            vec3 s_normalized = glm::normalize(s);
            float cos_theta = glm::max(0.0f, glm::dot(currentTriangleNormal_world, s_normalized));
            illumination_direct = (lightPower * cos_theta) / (4.0f * MY_PI * dist_sq);
        }
        
        vec3 final_color = currentColor * (illumination_direct + indirectLightPowerPerArea);
        final_color = glm::clamp(final_color, 0.0f, 1.0f);
        
        sdlAux->putPixelWithCapture(x, y, final_color);
    }
}

void Draw() {
    // Clear depth buffer and set sky gradient background
    for (int r = 0; r < SCREEN_HEIGHT; ++r) {
        for (int c = 0; c < SCREEN_WIDTH; ++c) {
            depthBuffer[r][c] = 0.0f;
            
            // Create white-blue gradient sky background
            // Top of screen (r=0) = light blue, bottom of screen (r=SCREEN_HEIGHT-1) = white
            float gradient_factor = static_cast<float>(r) / static_cast<float>(SCREEN_HEIGHT - 1);
            
            // Sky blue at top: (0.5, 0.7, 1.0), white at bottom: (1.0, 1.0, 1.0)
            vec3 sky_top = vec3(0.5f, 0.7f, 1.0f);    // Light blue
            vec3 sky_bottom = vec3(1.0f, 1.0f, 1.0f); // White
            
            // Linear interpolation between top and bottom colors
            vec3 sky_color = sky_top * (1.0f - gradient_factor) + sky_bottom * gradient_factor;
            
            // Set the background pixel
            sdlAux->putPixelWithCapture(c, r, sky_color);
        }
    }

    for (size_t i = 0; i < triangles.size(); ++i) {
        vector<Vertex> polygon_vertices(3);
        polygon_vertices[0].position = triangles[i].v0;
        polygon_vertices[1].position = triangles[i].v1;
        polygon_vertices[2].position = triangles[i].v2;
        
        currentTriangleNormal_world = triangles[i].normal;
        currentColor = triangles[i].color;

        DrawPolygonPixel(polygon_vertices);
    }

    sdlAux->render();
}

// === Main Function ===
int main(int argc, char* argv[]) {
    std::cout << "[VR Renderer-Streamer] Starting..." << std::endl;

    // Initialize renderer
    LoadTestModel(triangles);
    sdlAux = new ExtendedSDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
    t = SDL_GetTicks();
    R = mat3(1.0f);

    // Initialize streaming
    if (!initializeStreaming()) {
        std::cerr << "Failed to initialize streaming" << std::endl;
        delete sdlAux;
        return -1;
    }

    // Start pose receiving thread
    std::cout << "[VR Renderer-Streamer] Starting pose receiver thread..." << std::endl;
    pose_receiver_thread = std::thread(pose_receiver_thread_func);

    auto last_stream_time = std::chrono::high_resolution_clock::now();

    while (!sdlAux->quitEvent() && keep_running) {
        // Update scene (this now handles pose-driven camera)
        Update();
        
        // Render frame
        Draw();

        // Stream frame if enough time has passed
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stream_time).count();

        if (elapsed_ms >= SEND_INTERVAL_MS) {
            last_stream_time = now;
            streamFrame();
        }

        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Cleanup
    std::cout << "[VR Renderer-Streamer] Shutting down..." << std::endl;
    
    // Signal threads to stop
    keep_running = false;
    
    // Wait for pose receiver thread to finish
    if (pose_receiver_thread.joinable()) {
        std::cout << "[VR Renderer-Streamer] Waiting for pose receiver thread to finish..." << std::endl;
        pose_receiver_thread.join();
    }
    
    // Clean up resources
    sdlAux->saveBMP("screenshot.bmp");
    delete sdlAux;
    closesocket(image_sender_socket);
    
    std::cout << "[VR Renderer-Streamer] Terminated." << std::endl;
    return 0;
}
