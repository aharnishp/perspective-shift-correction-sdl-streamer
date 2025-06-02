//DH2323 skeleton code, Lab3 (SDL2 version)
#include <iostream>
#include <glm/glm.hpp>
// #include <glm/gtc/constants.hpp> // REMOVED for PI
#include "SDL2auxiliary.h"
#include "TestModel.h" // Assumed to provide Triangle struct with v0,v1,v2, color, and normal
#include <algorithm> //for max(), min()
#include <vector> // for std::vector
#include <cmath> // For M_PI if available and preferred, or just define PI manually

#define fori(i,n) for(int i=0; i<n; ++i)

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::ivec2; // Added for clarity, though often included by glm.hpp

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL2Aux *sdlAux;
int t;
vector<Triangle> triangles; // From TestModel.h
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

const float MY_PI = 3.141592653f;

// Camera
vec3 cameraPos(0, 0, -3.001);
mat3 R; // Camera rotation matrix
float yaw = 0.0f; // Rotation arround Y axis
float pitch = 0.0f; // Rotation around X axis
float focal = (SCREEN_HEIGHT + SCREEN_WIDTH) * (0.5f); // Focal length

// Lighting
vec3 lightPos(0, -0.5, -0.7); // World space light position
vec3 lightPower = 3.0f * vec3(1, 1, 1);
vec3 indirectLightPowerPerArea = 0.5f * vec3(1, 1, 1);

// Current triangle properties for shader
vec3 currentColor; // Reflectance (rho) of the current triangle (from triangle.color)
vec3 currentTriangleNormal_world; // World-space normal of the current triangle

struct Pixel {
    int x, y;                   // Screen coordinates
    float zinv;                 // Inverse of camera-space Z coordinate (1/z_cam)
    vec3 pos_world_times_zinv;  // World-space position * zinv (for perspective-correct interpolation)
                                // Equivalent to world_position / z_cam
};

struct Vertex {
    vec3 position; // World-space position of the vertex
};


void Update(void);
void Draw(void);


void VertexShaderPixel(const Vertex& v_in, Pixel& p_out);
void InterpolatePixel(Pixel a, Pixel b, vector<Pixel>& result);
void ComputePolygonRowsPixel(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels, int& minHeight, int& maxHeight);
void DrawLineHorizontalDepth(const Pixel& start, const Pixel& end); 
void DrawRowsPixel(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, int polygonMinY, int polygonMaxY); 
void DrawPolygonPixel(const vector<Vertex>& vertices);
void PixelShader(const Pixel& p);


// ----------------------------------------------------------------------------
// MAIN

int main(int argc, char* argv[]) {
    LoadTestModel(triangles);  
    sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
    t = SDL_GetTicks(); 

    R = mat3(1.0f); 

    while (!sdlAux->quitEvent())
    {
        Update();
        Draw();
    }
    sdlAux->saveBMP("screenshot.bmp");
    delete sdlAux;
    return 0;
}

// ----------------------------------------------------------------------------
// UPDATE LOGIC

void Update(void) {
    // Compute frame time:
    int t2 = SDL_GetTicks();
    float dt = float(t2 - t);
    t = t2;
    // cout << "Render time: " << dt << " ms." << endl; 

    
    int dx, dy;
    SDL_GetRelativeMouseState(&dx, &dy);
    if (true) { // Rotate if left mouse button is pressed
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
    if (keystate[SDL_SCANCODE_UP]) lightPos.z = lightMoveSpeed; 
    if (keystate[SDL_SCANCODE_DOWN]) lightPos.z = -lightMoveSpeed;
    if (keystate[SDL_SCANCODE_LEFT]) lightPos.x = -lightMoveSpeed;
    if (keystate[SDL_SCANCODE_RIGHT]) lightPos.x = lightMoveSpeed;
    if (keystate[SDL_SCANCODE_PAGEUP]) lightPos.y = lightMoveSpeed;
    if (keystate[SDL_SCANCODE_PAGEDOWN]) lightPos.y = -lightMoveSpeed;


    
    // R = R_pitch * R_yaw
    mat3 R_yaw = mat3(
        vec3(cos(yaw), 0, -sin(yaw)), // Note: -sin(yaw) for right-handed system if yaw is around Y looking down Z
        vec3(0, 1, 0),
        vec3(sin(yaw), 0, cos(yaw))
    );
     // If yaw is rotation around positive Y axis:
     //  cos(yaw)  0   sin(yaw)
     //  0         1   0
     // -sin(yaw)  0   cos(yaw)
     // This is for rotating a point. For transforming CS, it's the inverse.
     // Let's use the standard view matrix construction:
    R_yaw = glm::mat3(
        glm::vec3(cos(yaw), 0.0f, sin(yaw)),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(-sin(yaw), 0.0f, cos(yaw))
    );

    mat3 R_pitch = glm::mat3(
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, cos(pitch), -sin(pitch)),
        glm::vec3(0.0f, sin(pitch), cos(pitch))
    );
    R = R_pitch * R_yaw; // This order means yaw is applied first in world, then pitch in the new x-axis.
}

// ----------------------------------------------------------------------------
// VERTEX SHADER (Pixel version)

void VertexShaderPixel(const Vertex& v_in, Pixel& p_out) {
    // Transform vertex position from world space to camera space
    // P_camera = R_view * (P_world - CameraPos_world)
    vec3 position_camera_space = R * (v_in.position - cameraPos);


    // Basic clipping for points behind or too close to the near plane
    if (position_camera_space.z < 0.001f) { // Avoid division by zero or negative z
        p_out.x = -1; 
        p_out.y = -1;
        p_out.zinv = 0; 
        p_out.pos_world_times_zinv = vec3(0.0f); // Initialize to prevent garbage
        return;
    }

    p_out.zinv = 1.0f / position_camera_space.z;

    // Perspective projection to screen coordinates
    p_out.x = static_cast<int>(round(focal * position_camera_space.x * p_out.zinv + SCREEN_WIDTH / 2.0f));
    p_out.y = static_cast<int>(round(focal * position_camera_space.y * p_out.zinv + SCREEN_HEIGHT / 2.0f));
    
    p_out.pos_world_times_zinv = v_in.position * p_out.zinv;
}

// ----------------------------------------------------------------------------
// INTERPOLATION

void InterpolatePixel(Pixel a, Pixel b, vector<Pixel>& result) {
    int N = result.size();
    if (N == 0) return;
    if (N == 1) {
        result[0] = a;
        return;
    }

    float num_steps = static_cast<float>(N - 1);
    // Handle N-1 == 0 case to avoid division by zero if N=1 (already handled, but defensive)
    if (num_steps < 1e-5) { // Effectively N=1
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


// ----------------------------------------------------------------------------
// POLYGON RASTERIZATION LOGIC

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
    if (validVertices < 3 && !(validVertices == 2 && vertexPixels[0].y != vertexPixels[1].y)) { 
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

        if (p1.y == p2.y) continue; // Skip horizontal edges for this scanline filling method
        
        // Clip edge's y-range to polygon's overall y-range
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
                if (y < 0 || y >= SCREEN_HEIGHT) continue; // Should be redundant due to polygonMinY/MaxY clipping

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

        if (abs(dx_row) < 1e-5) { // Effectively dx_row is zero (single point or vertical segment)
            currentPixel.zinv = startNode.zinv;
            currentPixel.pos_world_times_zinv = startNode.pos_world_times_zinv;
        } else {
            float alpha = static_cast<float>(x - x_start) / static_cast<float>(dx_row);
            alpha = glm::clamp(alpha, 0.0f, 1.0f); // Ensure alpha is in [0,1] for robust interpolation
            currentPixel.zinv = glm::mix(startNode.zinv, endNode.zinv, alpha);
            currentPixel.pos_world_times_zinv = glm::mix(startNode.pos_world_times_zinv, endNode.pos_world_times_zinv, alpha);
        }
        PixelShader(currentPixel);
    }
}

void DrawRowsPixel(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, int polygonMinY, int polygonMaxY) {
    for (int y = polygonMinY; y <= polygonMaxY; ++y) {
        if (y < 0 || y >= SCREEN_HEIGHT) continue;

        // Check if the x values are valid (not the initial SCREEN_WIDTH or 0 from init if no edge touched this row)
        // A more robust check would be if leftPixels[y].zinv and rightPixels[y].zinv are non-zero,
        // assuming they are initialized to 0 and only set if an edge interpolates to this row.
        // For now, rely on x comparison.
        if (leftPixels[y].x <= rightPixels[y].x) { // Use <= to draw single-pixel-wide vertical lines
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

// ----------------------------------------------------------------------------
// PIXEL SHADER

void PixelShader (const Pixel& p){
    int x = p.x;
    int y = p.y;

    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) {
        return;
    }

    if (p.zinv > depthBuffer[y][x]) 
    {
        depthBuffer[y][x] = p.zinv; 
        
        vec3 actual_pixel_world_pos;
        if (abs(p.zinv) < 1e-9) { // Avoid division by zero or very small zinv
             // This pixel is likely very far or an error, paint with background or skip
             // For now, let's assume it's an error or too far to light meaningfully
             sdlAux->putPixel(x, y, vec3(0.1,0.1,0.1)); // Dim color
             return;
        } else {
            actual_pixel_world_pos = p.pos_world_times_zinv / p.zinv;
        }

        vec3 s = lightPos - actual_pixel_world_pos;
        float dist_sq = glm::dot(s, s); 

        vec3 illumination_direct(0.0f);

        if (dist_sq > 1e-5f) { // Check for very small distance squared to avoid division by zero
            vec3 s_normalized = glm::normalize(s);
            float cos_theta = glm::max(0.0f, glm::dot(currentTriangleNormal_world, s_normalized));
            illumination_direct = (lightPower * cos_theta) / (4.0f * MY_PI * dist_sq);
        }
        
        vec3 final_color = currentColor * (illumination_direct + indirectLightPowerPerArea);
        final_color = glm::clamp(final_color, 0.0f, 1.0f);
        
        sdlAux->putPixel(x, y, final_color); 
    }
}

// ----------------------------------------------------------------------------
// DRAW FUNCTION

void Draw() {
    sdlAux->clearPixels(); 

    for (int r = 0; r < SCREEN_HEIGHT; ++r) {
        for (int c = 0; c < SCREEN_WIDTH; ++c) {
            depthBuffer[r][c] = 0.0f; 
        }
    }

    for (size_t i = 0; i < triangles.size(); ++i)
    {
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
