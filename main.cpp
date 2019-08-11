#include "nanort.h"
#include "CoconutAle/math.h"
#include "SDL.h"

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// windows specific stuff
#if defined(_MSC_VER)
#include <windows.h>
#include <shellapi.h>

#define and &&
#define or ||
#define not !

#define debug_print(...) \
{\
    static char print_buffer [100];\
    sprintf(print_buffer, __VA_ARGS__);\
    OutputDebugString(print_buffer);\
}

#else

#define debug_print(...) printf(...)

#endif

#include <iostream>
#include <fstream>

struct RenderObject {
    // TODO convert all to ca::Vec3f
    std::vector<float> verts;
    std::vector<nanort::real3<> > normals;
    std::vector<unsigned int> faces;
};

RenderObject bunny;

ca::Vec3f eye = {0.0f, 0.0f, -1.3f};
ca::Vec3f forward;
ca::Vec3f right;

ca::Mat3f look_matrix;

float walk_speed = 0.1f;
// TODO pitch yaw blah blah
float xAngle = 0.f;
float yAngle = 0.f;

void update_look_matrix() {
    // calculate rotation
    ca::Quat xQuat = ca::axis_angle_quat({1.0f,0.0f,0.0f}, xAngle);
    ca::Quat yQuat = ca::axis_angle_quat({0.0f,1.0f,0.0f}, yAngle);
    ca::Quat total_rotation = xQuat * yQuat;

    // re-calculate matrices and look vectors
    look_matrix = ca::RotationMat3f(total_rotation);
    // TODO can't we extract the vectors from the look matrix?
    forward = ca::mat_vec_mult(look_matrix, forward);
    forward.y = 0;
    normalize_modify(forward);
    right = ca::mat_vec_mult(look_matrix, right);
    right.y = 0;
    normalize_modify(right);
}

void render_scene(
    int height,
    int width,
    const nanort::BVHAccel<float> & accel,
    const nanort::TriangleIntersector<> & intersector,
    SDL_Surface * target)
{
    SDL_LockSurface(target);
    unsigned char * target_pixels = (unsigned char *)target->pixels;
    // Shoot rays.
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    const float tFar = 1.0e+30f;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            nanort::BVHTraceOptions trace_options;
            // Simple camera. change eye pos and direction fit to .obj model. 
            ca::Vec3f ray_dir = {
                (x / (float)width) - 0.5f,
                (y / (float)height) - 0.5f,
                1.0f};
            ca::Vec3f rotated_vector = ca::mat_vec_mult(look_matrix, ray_dir);
            nanort::Ray<float> ray;
            ray.min_t = 0.0f;
            ray.max_t = tFar;
            ray.org[0] = eye.x;
            ray.org[1] = eye.y;
            ray.org[2] = eye.z;
            ray.dir[0] = rotated_vector.x;
            ray.dir[1] = rotated_vector.y;
            ray.dir[2] = rotated_vector.z;
            nanort::TriangleIntersection<> isect;
            bool hit = accel.Traverse(ray, intersector, &isect, trace_options);
            unsigned char * pixels = &(target_pixels[x * 4 + y * (width * 4)]);
            if (hit) {
                // TODO Write your shader here.
                // TODO rename fid to something else
                unsigned int fid = isect.prim_id;
                nanort::real3<> normal = bunny.normals[fid];
                static nanort::real3<> negZ(0.0f, 0.0f, 1.0f);
                float red_color = nanort::vlength(normal * negZ) * 250 + 5.0f;
                // TODO store normals
                // normal[0] = mesh.facevarying_normals[3*3*fid+0];
                // normal[1] = mesh.facevarying_normals[3*3*fid+1];
                // normal[2] = mesh.facevarying_normals[3*3*fid+2];
                pixels[0] = red_color;
                pixels[1] = 0;
                pixels[2] = 0;
            } else {
                pixels[0] = 0;
                pixels[1] = 0;
                pixels[2] = 0;
            }
            pixels[3] = 255;
        }
    }
    SDL_UnlockSurface(target);
}

void debug_function(
    int height,
    int width,
    const nanort::BVHAccel<float> & accel,
    const nanort::TriangleIntersector<> & intersector,
    SDL_Surface * target) {
    static ca::Vec3f eye_arr [5] = {
        {0.0f, 0.0f, -0.3f},
        {0.3f, 0.0f, -0.3f},
        {-0.3f, 0.0f, -0.3f},
        {0.0f, 0.3f, -0.3f},
        {0.0f, -0.3f, -0.3f}
    };
    static unsigned eye_ind = 0;
    eye = eye_arr[eye_ind];
    eye_ind++;
    if (eye_ind > 5) {
        eye_ind = 0;
    }
    render_scene(height, width, accel, intersector, target);
}

int window_scale = 2;
int width = 64, height = 64;

// SDL helper stuff
struct SDLWindowSurfacePair
{
    SDL_Window * mainWindow;
    SDL_Surface * screenSurface;
};

SDLWindowSurfacePair
SDL_init_window()
{
    SDL_Window * mainWindow;
    SDL_Surface * screenSurface;
    // Initialize SDL window
    {

        //Initialization flag
        bool success = true;

        //Initialize SDL
        if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
        {
            printf( "SDL could not initialize! SDL Error: %s\n",
                SDL_GetError());
            success = false;
        }
        else
        {
            //Create window
            mainWindow = SDL_CreateWindow(
                "SDL Tutorial",
                SDL_WINDOWPOS_UNDEFINED,
                SDL_WINDOWPOS_UNDEFINED,
                window_scale * width,
                window_scale * height,
                SDL_WINDOW_SHOWN);
            if( mainWindow == NULL )
            {
                printf("Window could not be created! SDL Error: %s\n",
                    SDL_GetError());
                success = false;
            }
            else
            {
                //Get window surface
                screenSurface = SDL_GetWindowSurface( mainWindow );
            }
        }

    }
    return {mainWindow, screenSurface};
}

SDL_Surface *
SDL_rendered_surface_init()
{
    //Main loop flag
    bool quit = false;
    // create the render target
    int rmask;
    int gmask;
    int bmask;
    int amask;

#if SDL_BYTEORDER == SDL_BIG_ENDIAN
    rmask = 0xff000000;
    gmask = 0x00ff0000;
    bmask = 0x0000ff00;
    amask = 0x000000ff;
#else
    rmask = 0x000000ff;
    gmask = 0x0000ff00;
    bmask = 0x00ff0000;
    amask = 0xff000000;
#endif


    return SDL_CreateRGBSurface(0, /* flags */
            width, height,
            32, /* bitdepth */
            rmask, gmask, bmask, amask);
}

#if defined(_MSC_VER)
#define PROG_MAIN int WINAPI WinMain(HINSTANCE, HINSTANCE, LPTSTR, int)
#else
#define PROG_MAIN int main(int argc, char ** argv)
#endif

PROG_MAIN {
    std::string line;
    std::ifstream myFile ("bunny.obj");
    unsigned numVerts, numFaces;
    unsigned num_copies = 4;
    if (myFile.is_open()) {
        myFile >> numVerts >> numFaces;
        bunny.verts.resize(numVerts * 3 * num_copies);
        bunny.normals.resize(numFaces * num_copies);
        bunny.faces.resize(numFaces * 3 * num_copies);
        for (unsigned i = 0; i < numVerts; i++) {
            char v;
            float v1, v2, v3;
            myFile >> v >> v1 >> v2 >> v3;
            for (int j = 0; j < num_copies; j++) {
                bunny.verts[i + j * numVerts + 0] = v1 + j * 2;
                bunny.verts[i + j * numVerts + 1] = v2 + 0;
                bunny.verts[i + j * numVerts + 2] = v3 + 0;
            }
        }
        for (unsigned i = 0; i < numFaces; i++) {
            char f;
            int f1, f2, f3;
            myFile >> f >> f1 >> f2 >> f3;

            // TODO switch to use ca::Vec3f
            nanort::real3<> p1 (&bunny.verts[f1]);
            nanort::real3<> p2 (&bunny.verts[f2]);
            nanort::real3<> p3 (&bunny.verts[f3]);
            nanort::real3<> u = p2 - p1;
            nanort::real3<> v = p3 - p1;

            nanort::real3<> normal;
            normal[0] = u.y() * v.z() - u.z() * v.y();
            normal[1] = u.z() * v.x() - u.x() * v.z();
            normal[2] = u.x() * v.y() - u.y() * v.x();

            for (int j = 0; j < num_copies; j++) {
                bunny.faces[i + j * numFaces + 0] = f1 + j * numVerts;
                bunny.faces[i + j * numFaces + 1] = f2 + j * numVerts;
                bunny.faces[i + j * numFaces + 2] = f3 + j * numVerts;

                bunny.normals[i + j * numFaces] = nanort::vnormalize(normal);
            }
        }
    } else {
        // TODO exit
    }

    // initialize global values
    forward = {0.0f, 0.0f, 1.0f};
    right   = {1.0f, 0.0f, 0.0f};
    update_look_matrix();

    nanort::BVHBuildOptions<float> options;
    nanort::TriangleMesh<float> triangle_mesh(
            bunny.verts.data(),
            bunny.faces.data(),
            sizeof(float) * 3/* stride */);
    nanort::TriangleSAHPred<float> triangle_pred(
            bunny.verts.data(),
            bunny.faces.data(),
            sizeof(float) * 3/* stride */);

    nanort::TriangleIntersector<> triangle_intersecter(
            bunny.verts.data(),
            bunny.faces.data(),
            sizeof(float) * 3/* stride */);
    nanort::BVHAccel<float> accel;
    accel.Build(numFaces * num_copies, triangle_mesh, triangle_pred, options);
    nanort::BVHBuildStatistics stats = accel.GetStatistics();
    debug_print("  BVH statistics:\n");
    debug_print("    # of leaf   nodes: %d\n", stats.num_leaf_nodes);
    debug_print("    # of branch nodes: %d\n", stats.num_branch_nodes);
    debug_print("  Max tree depth   : %d\n", stats.max_tree_depth);

    // Initialize SDL

    SDLWindowSurfacePair sdl_init_result = SDL_init_window();
    SDL_Window * mainWindow = sdl_init_result.mainWindow;
    SDL_Surface * screenSurface = sdl_init_result.screenSurface;

    // SDL loop
    {
        SDL_Surface * renderedSurface = SDL_rendered_surface_init();
        render_scene(width, height, accel, triangle_intersecter, renderedSurface);

        if (SDL_BlitScaled( renderedSurface, NULL, screenSurface, NULL )) {
            printf("ERROR>>> %s\n", SDL_GetError());
        }

        SDL_Event e;
        bool quit = false;
        //While application is running
        while( !quit )
        {
            if ( SDL_PollEvent( &e ) != 0 )
            {
                do
                {
                    if( e.type == SDL_KEYDOWN )
                    {
                        //Select surfaces based on key press
                        switch( e.key.keysym.sym ) {
                            case SDLK_ESCAPE:
                                quit = true;
                                continue;
                            case SDLK_w:
                                eye += (forward * walk_speed);
                                break;
                            case SDLK_a:
                                eye -= (right * walk_speed);
                                break;
                            case SDLK_s:
                                eye -= (forward * walk_speed);
                                break;
                            case SDLK_d:
                                eye += (right * walk_speed);
                                break;
                            case SDLK_LEFT:
                                yAngle += 0.1f;
                                update_look_matrix();
                                break;

                            case SDLK_RIGHT:
                                yAngle -= 0.1f;
                                update_look_matrix();
                                break;
                            case SDLK_UP:
                                xAngle -= 0.1f;
                                update_look_matrix();
                                break;

                            case SDLK_DOWN:
                                xAngle += 0.1f;
                                update_look_matrix();
                                break;
                        }
                    }
                    else if( e.type == SDL_QUIT )
                    {
                        quit = true;
                        continue;
                    }
                } while ( SDL_PollEvent( &e ) != 0 );
                render_scene(width, height, accel, triangle_intersecter, renderedSurface);
                if (SDL_BlitScaled( renderedSurface, NULL, screenSurface, NULL )) {
                    printf("ERROR>>> %s\n", SDL_GetError());
                }
            }
            SDL_UpdateWindowSurface(mainWindow);
        }
    }

    return 0;
}
