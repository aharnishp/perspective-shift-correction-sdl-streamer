#ifndef EXTENDED_SDL2_AUX_H
#define EXTENDED_SDL2_AUX_H

#include <glm/glm.hpp>
#include "SDL2Auxiliary/SDL2Auxiliary.h"
#include <vector>

class ExtendedSDL2Aux : public SDL2Aux {
private:
    std::vector<unsigned char> rgb_buffer;
    
public:
    ExtendedSDL2Aux(int width, int height, bool fullscreen = false);
    void extractRGBBuffer(std::vector<unsigned char>& buffer);
    void putPixelWithCapture(int x, int y, glm::vec3 color);
    
private:
    int screen_width;
    int screen_height;
};

#endif
