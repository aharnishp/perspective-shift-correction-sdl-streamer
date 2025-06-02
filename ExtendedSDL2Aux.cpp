#include "ExtendedSDL2Aux.h"
#include <cstring>
#include <glm/glm.hpp>

ExtendedSDL2Aux::ExtendedSDL2Aux(int width, int height, bool fullscreen) 
    : SDL2Aux(width, height, fullscreen), screen_width(width), screen_height(height) {
    rgb_buffer.resize(width * height * 3, 0);
}

void ExtendedSDL2Aux::putPixelWithCapture(int x, int y, glm::vec3 color) {
    // Call the parent putPixel method to maintain normal rendering
    SDL2Aux::putPixel(x, y, color);
    
    // Also store the RGB values in our buffer for streaming
    if (x >= 0 && x < screen_width && y >= 0 && y < screen_height) {
        int idx = (y * screen_width + x) * 3;
        
        // Convert from [0,1] to [0,255] and clamp
        unsigned char r = static_cast<unsigned char>(glm::clamp(color.r * 255.0f, 0.0f, 255.0f));
        unsigned char g = static_cast<unsigned char>(glm::clamp(color.g * 255.0f, 0.0f, 255.0f));
        unsigned char b = static_cast<unsigned char>(glm::clamp(color.b * 255.0f, 0.0f, 255.0f));
        
        rgb_buffer[idx] = r;
        rgb_buffer[idx + 1] = g;
        rgb_buffer[idx + 2] = b;
    }
}

void ExtendedSDL2Aux::extractRGBBuffer(std::vector<unsigned char>& buffer) {
    buffer = rgb_buffer;
}
