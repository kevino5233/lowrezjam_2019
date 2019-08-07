#include "nanort.h"
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
            nanort::Ray<float> ray;
            ray.min_t = 0.0f;
            ray.max_t = tFar;
            // TODO camera position
            ray.org[0] = 0.0f;
            ray.org[1] = 0.0f;
            ray.org[2] = 1.0f;
            nanort::real3<> dir;
            dir[0] = (x / (float)width) - 0.5f;
            dir[1] = (y / (float)height) - 0.5f;
            dir[2] = -1.0f;
            dir = vnormalize(dir);
            ray.dir[0] = dir[0];
            ray.dir[1] = dir[1];
            ray.dir[2] = dir[2];
            nanort::TriangleIntersection<> isect;
            bool hit = accel.Traverse(ray, intersector, &isect, trace_options);
            unsigned char * pixels = &(target_pixels[x * 4 + y * (width * 4)]);
            if (hit) {
                // TODO Write your shader here.
                debug_print("hit!\n");
                // nanort::real3<> normal;
                // unsigned int fid = isect.prim_id;
                // TODO store normals
                // normal[0] = mesh.facevarying_normals[3*3*fid+0];
                // normal[1] = mesh.facevarying_normals[3*3*fid+1];
                // normal[2] = mesh.facevarying_normals[3*3*fid+2];
                pixels[0] = 255;
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
    // TODO get normals
    std::vector<float> bunnyVerts;
    std::vector<unsigned int> bunnyFaces;
    int numVerts, numFaces;
    if (myFile.is_open()) {
        myFile >> numVerts >> numFaces;
        bunnyVerts.reserve(numVerts);
        bunnyFaces.reserve(numFaces);
        for (int i = 0; i < numVerts; i++) {
            char v;
            float v1, v2, v3;
            myFile >> v >> v1 >> v2 >> v3;
            bunnyVerts.push_back(v1);
            bunnyVerts.push_back(v2);
            bunnyVerts.push_back(v3);
        }
        for (int i = 0; i < numFaces; i++) {
            char f;
            int f1, f2, f3;
            myFile >> f >> f1 >> f2 >> f3;
            bunnyFaces.push_back(f1);
            bunnyFaces.push_back(f2);
            bunnyFaces.push_back(f3);
        }
    } else {
        // TODO exit
    }
    nanort::BVHBuildOptions<float> options;
    nanort::TriangleMesh<float> triangle_mesh(
            bunnyVerts.data(),
            bunnyFaces.data(),
            sizeof(float) * 3/* stride */);
    nanort::TriangleSAHPred<float> triangle_pred(
            bunnyVerts.data(),
            bunnyFaces.data(),
            sizeof(float) * 3/* stride */);

    nanort::TriangleIntersector<> triangle_intersecter(
            bunnyVerts.data(),
            bunnyFaces.data(),
            sizeof(float) * 3/* stride */);
    nanort::BVHAccel<float> accel;
    accel.Build(bunnyFaces.size() / 3, triangle_mesh, triangle_pred, options);
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
                    if( e.type == SDL_QUIT )
                    {
                        quit = true;
                        continue;
                    }
                } while ( SDL_PollEvent( &e ) != 0 );
            }
            SDL_UpdateWindowSurface(mainWindow);
        }
    }

    return 0;
}
