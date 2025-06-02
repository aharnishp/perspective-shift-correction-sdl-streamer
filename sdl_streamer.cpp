#include <iostream>
#include <vector>
#include <string>
#include <cstring> // For memset, memcpy
#include <chrono>
#include <thread>   // For std::this_thread::sleep_for
#include <atomic>   // For atomic_bool (though can be simplified if not threading)
#include <cstdint>  // For uint16_t, uint32_t
#include <algorithm> // For std::min

// Networking includes (assuming Linux/macOS, adjust for Windows if needed)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> // For close()

// Cross-platform socket definitions (simplified for non-Windows)
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close

// STB Image Write include
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h" // Needs to be in your include path

// === Structures ===
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
    // Prevent copying
    MemoryBuffer(const MemoryBuffer&) = delete;
    MemoryBuffer& operator=(const MemoryBuffer&) = delete;
};

// Callback for stb_image_write to write to our MemoryBuffer
void write_memory_buffer(void* context, void* data, int size) {
    if (!context || !data || size <= 0) return;

    MemoryBuffer* buffer = static_cast<MemoryBuffer*>(context);
    size_t new_size = buffer->size + size;

    if (new_size > buffer->capacity) {
        size_t new_capacity = (buffer->capacity == 0) ?
            std::max(static_cast<size_t>(size * 2), static_cast<size_t>(4096)) : // Start with reasonable size
            std::max(buffer->capacity * 2, new_size);

        unsigned char* new_data = static_cast<unsigned char*>(
            buffer->data ? realloc(buffer->data, new_capacity) :
                         malloc(new_capacity));

        if (!new_data) {
            // Handle allocation failure if necessary, e.g., throw or log
            return;
        }
        buffer->data = new_data;
        buffer->capacity = new_capacity;
    }

    memcpy(buffer->data + buffer->size, data, size);
    buffer->size = new_size;
}

#pragma pack(push, 1)
struct PacketHeader {
    uint32_t frameId;
    uint32_t chunkIndex;
    uint32_t totalChunks;
    uint8_t dataType; // 0 for JPEG
};
#pragma pack(pop)

// === Configuration ===
const unsigned int IMG_WIDTH = 512;  // Image width
const unsigned int IMG_HEIGHT = 512; // Image height
const int NUM_CHANNELS = 3;          // Assuming RGB images

const char* TARGET_IP = "127.0.0.1"; // IP address of the receiver
const unsigned short TARGET_PORT = 8051;  // Port of the receiver
const int JPEG_QUALITY = 75;             // JPEG compression quality (1-100)
const int SEND_INTERVAL_MS = 33;         // Approx. 30 FPS

// Fragmentation settings
const int MAX_UDP_PACKET_SIZE = 1400;    // Max UDP payload size
const int HEADER_SIZE = sizeof(PacketHeader); // Size of our custom header
// const int DATA_TYPE_HEADER_SIZE = 1; // Included in PacketHeader's dataType
const int MAX_CHUNK_DATA_SIZE = MAX_UDP_PACKET_SIZE - HEADER_SIZE;

// Global control
std::atomic<bool> keep_running(true);

// === Helper Functions ===
void print_socket_error(const std::string& message) {
    perror(message.c_str());
}

// Dummy function to simulate acquiring image data
// In a real application, this would get data from a camera, file, or other source.
// Fills the buffer with a simple gradient pattern.
void populate_image_buffer(std::vector<unsigned char>& buffer, unsigned int width, unsigned int height, int channels, uint32_t frame_id) {
    buffer.resize(width * height * channels);
    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                // Simple gradient based on position and frame_id to make it dynamic
                buffer[(y * width + x) * channels + c] = (unsigned char)((x + y + frame_id + c * 64) % 256);
            }
        }
    }
}


// === Main Application ===
int main() {
    std::cout << "[Image Streamer] Starting..." << std::endl;

    // --- Setup Networking (Image Sender) ---
    SOCKET image_sender_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (image_sender_socket == INVALID_SOCKET) {
        print_socket_error("Failed to create image sender socket");
        return -1;
    }

    sockaddr_in receiver_addr;
    memset(&receiver_addr, 0, sizeof(receiver_addr));
    receiver_addr.sin_family = AF_INET;
    receiver_addr.sin_port = htons(TARGET_PORT);
    if (inet_pton(AF_INET, TARGET_IP, &receiver_addr.sin_addr) <= 0) {
        std::cerr << "Invalid target IP address: " << TARGET_IP << std::endl;
        closesocket(image_sender_socket);
        return -1;
    }
    std::cout << "[Image Sender] Ready to send to " << TARGET_IP << ":" << TARGET_PORT << std::endl;

    // Buffer for raw pixel data
    std::vector<unsigned char> pixel_buffer(IMG_WIDTH * IMG_HEIGHT * NUM_CHANNELS);

    // --- Sending Loop ---
    uint32_t frame_count = 0;
    auto last_send_time = std::chrono::high_resolution_clock::now();

    // Buffer for a single UDP packet
    std::vector<unsigned char> packet_buffer_vec(MAX_UDP_PACKET_SIZE);


    while (keep_running) { // You can set keep_running to false via a signal handler or another thread for graceful shutdown
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time).count();

        if (elapsed_ms >= SEND_INTERVAL_MS) {
            last_send_time = now;
            frame_count++;

            // 1. Populate image buffer with new data
            populate_image_buffer(pixel_buffer, IMG_WIDTH, IMG_HEIGHT, NUM_CHANNELS, frame_count);

            // 2. Encode JPEG
            MemoryBuffer jpeg_buffer;
            // If your populate_image_buffer provides data in top-to-bottom order,
            // you don't need to flip. If it's bottom-to-top (like glReadPixels), enable flip.
            // stbi_flip_vertically_on_write(1); // Assuming populate_image_buffer is top-to-bottom
            int jpeg_success = stbi_write_jpg_to_func(
                write_memory_buffer,
                &jpeg_buffer,
                IMG_WIDTH, IMG_HEIGHT,
                NUM_CHANNELS,       // Number of color components (e.g., 3 for RGB)
                pixel_buffer.data(),
                JPEG_QUALITY
            );
            // stbi_flip_vertically_on_write(0); // Reset flip if it was set

            if (jpeg_success && jpeg_buffer.data && jpeg_buffer.size > 0) {
                // 3. Send JPEG with fragmentation
                uint32_t total_chunks = (jpeg_buffer.size + MAX_CHUNK_DATA_SIZE - 1) / MAX_CHUNK_DATA_SIZE;
                if (total_chunks == 0 && jpeg_buffer.size > 0) total_chunks = 1; // Handle cases smaller than MAX_CHUNK_DATA_SIZE

                PacketHeader* header = reinterpret_cast<PacketHeader*>(packet_buffer_vec.data());
                header->frameId = htonl(frame_count); // Network byte order for multi-byte integers
                header->totalChunks = htonl(total_chunks);
                header->dataType = 0; // 0 for JPEG

                size_t bytes_remaining = jpeg_buffer.size;
                const unsigned char* data_ptr = jpeg_buffer.data;

                // std::cout << "Frame " << frame_count << ": Sending " << jpeg_buffer.size << " bytes in " << total_chunks << " chunks." << std::endl;

                for (uint32_t chunk_idx = 0; chunk_idx < total_chunks; chunk_idx++) {
                    header->chunkIndex = htonl(chunk_idx); // Network byte order

                    int current_chunk_data_size = std::min(static_cast<size_t>(MAX_CHUNK_DATA_SIZE), bytes_remaining);
                    
                    if (current_chunk_data_size <= 0 && bytes_remaining > 0) { // Should not happen if logic is correct
                        std::cerr << "Warning: trying to send 0 byte data chunk for frame " << frame_count << ", chunk " << chunk_idx << std::endl;
                        continue;
                    }
                    if (current_chunk_data_size == 0 && bytes_remaining == 0 && total_chunks > 0 && chunk_idx == 0) { // Empty JPEG?
                         // Send at least one packet if jpeg_buffer.size was 0 but success was true
                         // This case is unlikely for valid JPEGs but handles empty buffer.
                    }


                    // Copy data payload after the header
                    memcpy(packet_buffer_vec.data() + HEADER_SIZE, data_ptr, current_chunk_data_size);

                    int packet_size = HEADER_SIZE + current_chunk_data_size;

                    int bytes_sent = sendto(image_sender_socket,
                        packet_buffer_vec.data(),
                        packet_size,
                        0,
                        reinterpret_cast<const struct sockaddr*>(&receiver_addr),
                        sizeof(receiver_addr));

                    if (bytes_sent == SOCKET_ERROR) {
                        print_socket_error("sendto (JPEG) failed");
                        // Optionally break or implement retry logic
                        break;
                    } else if (bytes_sent != packet_size) {
                        std::cerr << "Warning: sent " << bytes_sent << " bytes, but expected " << packet_size << std::endl;
                    }


                    data_ptr += current_chunk_data_size;
                    bytes_remaining -= current_chunk_data_size;

                    // Small delay between chunks to help receiver cope, especially if many chunks
                    if (total_chunks > 1 && chunk_idx < total_chunks - 1) {
                        std::this_thread::sleep_for(std::chrono::microseconds(100)); // Adjust as needed
                    }
                }
            } else {
                std::cerr << "Failed to encode JPEG frame " << frame_count << std::endl;
            }
        } else {
            // Sleep for a short duration if not sending to avoid busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(0, SEND_INTERVAL_MS - (int)elapsed_ms)));
        }

        // Add a condition to break the loop if needed, e.g., after N frames for testing
        // if (frame_count > 300) keep_running = false;
    }

    // --- Cleanup ---
    std::cout << "[Image Streamer] Loop exited. Cleaning up..." << std::endl;
    closesocket(image_sender_socket);
    std::cout << "[Image Sender] Socket closed." << std::endl;
    std::cout << "[Image Streamer] Terminated." << std::endl;

    return 0;
}