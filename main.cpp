#include <iostream>
#include <glm/glm.hpp>
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

// Networking includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

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
    uint8_t dataType; // 0 for JPEG
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
const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
const char* TARGET_IP = "192.168.31.144";
// const char* TARGET_IP = "127.0.0.1";
const unsigned short TARGET_PORT = 8051;
const int JPEG_QUALITY = 75;
const int SEND_INTERVAL_MS = 33; // ~30 FPS
const int MAX_UDP_PACKET_SIZE = 1400;
const int HEADER_SIZE = sizeof(PacketHeader);
const int MAX_CHUNK_DATA_SIZE = MAX_UDP_PACKET_SIZE - HEADER_SIZE;
const float MY_PI = 3.141592653f;

// Depth buffer configuration
const float DEPTH_NEAR = 0.1f;    // Near clipping plane
const float DEPTH_FAR = 100.0f;   // Far clipping plane
const float DEPTH_SCALE = 1000.0f; // Convert to millimeters

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
float focal = (SCREEN_HEIGHT + SCREEN_WIDTH) * (0.5f);

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
            float zinv = depthBuffer[y][x];
            
            if (zinv <= 0.0f) {
                // No depth or behind camera - set to 0 (error/infinite)
                depth_16bit[idx] = 0;
            } else {
                // Convert from 1/z to actual depth distance
                float depth_world = 1.0f / zinv;
                
                // Convert to millimeters and clamp to 16-bit range
                uint32_t depth_mm = static_cast<uint32_t>(depth_world * DEPTH_SCALE);
                depth_16bit[idx] = static_cast<uint16_t>(std::min(depth_mm, static_cast<uint32_t>(65535)));
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

bool sendDataPackets(const unsigned char* data, size_t data_size, uint8_t data_type) {
    uint32_t total_chunks = (data_size + MAX_CHUNK_DATA_SIZE - 1) / MAX_CHUNK_DATA_SIZE;
    if (total_chunks == 0 && data_size > 0) total_chunks = 1;

    std::vector<unsigned char> packet_buffer(MAX_UDP_PACKET_SIZE);
    PacketHeader* header = reinterpret_cast<PacketHeader*>(packet_buffer.data());
    
    header->frameId = htonl(frame_count);
    header->totalChunks = htonl(total_chunks);
    header->dataType = data_type; // 0 for JPEG, 1 for depth

    size_t bytes_remaining = data_size;
    const unsigned char* data_ptr = data;

    for (uint32_t chunk_idx = 0; chunk_idx < total_chunks; chunk_idx++) {
        header->chunkIndex = htonl(chunk_idx);

        int current_chunk_data_size = std::min(static_cast<size_t>(MAX_CHUNK_DATA_SIZE), bytes_remaining);
        
        memcpy(packet_buffer.data() + HEADER_SIZE, data_ptr, current_chunk_data_size);
        int packet_size = HEADER_SIZE + current_chunk_data_size;

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

        data_ptr += current_chunk_data_size;
        bytes_remaining -= current_chunk_data_size;

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
    if (true) {
        if (dx != 0) {
            yaw -= dx * -0.005f;
        }
        if (dy != 0) {
            pitch -= dy * 0.005f;
            const float pitchLimit = MY_PI / 2.0f - 0.01f;
            pitch = glm::clamp(pitch, -pitchLimit, pitchLimit);
        }
    }

    const Uint8* keystate = SDL_GetKeyboardState(NULL);
    float moveSpeed = 0.005f * dt;

    mat3 cameraToWorld = glm::transpose(R);
    vec3 forward_cam_local = vec3(0,0,-1);
    vec3 right_cam_local = vec3(1,0,0);

    vec3 forward_world = cameraToWorld * forward_cam_local;
    vec3 right_world = cameraToWorld * right_cam_local;

    if (keystate[SDL_SCANCODE_W]) cameraPos -= forward_world * moveSpeed;
    if (keystate[SDL_SCANCODE_S]) cameraPos += forward_world * moveSpeed;
    if (keystate[SDL_SCANCODE_A]) cameraPos -= right_world * moveSpeed;
    if (keystate[SDL_SCANCODE_D]) cameraPos += right_world * moveSpeed;
    if (keystate[SDL_SCANCODE_SPACE]) cameraPos.y += moveSpeed;
    if (keystate[SDL_SCANCODE_LCTRL]) cameraPos.y -= moveSpeed;

    float lightMoveSpeed = 0.005f * dt;
    if (keystate[SDL_SCANCODE_UP]) lightPos.z += lightMoveSpeed;
    if (keystate[SDL_SCANCODE_DOWN]) lightPos.z -= lightMoveSpeed;
    if (keystate[SDL_SCANCODE_LEFT]) lightPos.x -= lightMoveSpeed;
    if (keystate[SDL_SCANCODE_RIGHT]) lightPos.x += lightMoveSpeed;
    if (keystate[SDL_SCANCODE_PAGEUP]) lightPos.y += lightMoveSpeed;
    if (keystate[SDL_SCANCODE_PAGEDOWN]) lightPos.y -= lightMoveSpeed;

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
    R = R_pitch * R_yaw;
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
    sdlAux->clearPixels();

    for (int r = 0; r < SCREEN_HEIGHT; ++r) {
        for (int c = 0; c < SCREEN_WIDTH; ++c) {
            depthBuffer[r][c] = 0.0f;
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

    auto last_stream_time = std::chrono::high_resolution_clock::now();

    while (!sdlAux->quitEvent() && keep_running) {
        // Update scene
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
    sdlAux->saveBMP("screenshot.bmp");
    delete sdlAux;
    closesocket(image_sender_socket);
    
    std::cout << "[VR Renderer-Streamer] Terminated." << std::endl;
    return 0;
}
