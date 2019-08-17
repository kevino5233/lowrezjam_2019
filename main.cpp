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
    std::vector<ca::Vec3f> verts;
    std::vector<ca::Vec3f> normals;
    std::vector<ca::Vec3u> faces;
};

RenderObject bunny;
RenderObject squares;

ca::Vec3f eye = {0.0f, 0.0f, -2.3f};
ca::Vec3f forward;
ca::Vec3f right;

ca::Mat3f look_matrix;

float walk_speed = 0.1f;

void update_look_matrix(float xAngleDelta, float yAngleDelta) {
    // calculate rotation
    ca::Quat q_x = ca::axis_angle_quat({1.0f,0.0f,0.0f}, xAngleDelta);
    ca::Quat q_y = ca::axis_angle_quat({0.0f,1.0f,0.0f}, yAngleDelta);
    ca::Quat q_rotation = q_y * q_x;
    ca::Mat3f m_rotation = ca::RotationMat3f(q_rotation);
    look_matrix = m_rotation * look_matrix;
    ca::Vec3f * v3f_matrix_ptr = reinterpret_cast<ca::Vec3f *>(&look_matrix);

    // TODO can't we extract the vectors from the look matrix?
	forward = m_rotation * forward;
    forward.y = 0;
	right = m_rotation * right;
    right.y = 0;
}

struct NanortRenderData
{
    nanort::TriangleMesh<float> * mesh;
    nanort::TriangleSAHPred<float> * pred;
    nanort::TriangleIntersector<> * intersector;
    nanort::BVHAccel<float> * accel;
};

NanortRenderData
build_scene(const RenderObject &ro,
            const nanort::BVHBuildOptions<float> &options)
{
    NanortRenderData out;
    out.mesh = new nanort::TriangleMesh<float>(
            reinterpret_cast<const float *>(ro.verts.data()),
            reinterpret_cast<const unsigned *>(ro.faces.data()),
            sizeof(float) * 3/* stride */);
    out.pred = new nanort::TriangleSAHPred<float>(
            reinterpret_cast<const float *>(ro.verts.data()),
            reinterpret_cast<const unsigned *>(ro.faces.data()),
            sizeof(float) * 3/* stride */);

    out.intersector = new nanort::TriangleIntersector<>(
            reinterpret_cast<const float *>(ro.verts.data()),
            reinterpret_cast<const unsigned *>(ro.faces.data()),
            sizeof(float) * 3/* stride */);
    out.accel = new nanort::BVHAccel<float>;
    out.accel->Build(ro.faces.size(), *out.mesh, *out.pred, options);
    nanort::BVHBuildStatistics stats = out.accel->GetStatistics();
    debug_print("  BVH statistics:\n");
    debug_print("%zu\n", ro.faces.size());
    debug_print("    # of leaf   nodes: %d\n", stats.num_leaf_nodes);
    debug_print("    # of branch nodes: %d\n", stats.num_branch_nodes);
    debug_print("  Max tree depth   : %d\n", stats.max_tree_depth);
    return out;
}

void render_scene(
    int height,
    int width,
    const NanortRenderData &render_data,
    // const nanort::BVHAccel<float> & accel,
    // const nanort::TriangleIntersector<> & intersector,
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
            bool hit = render_data.accel->Traverse(
                ray, *render_data.intersector, &isect, trace_options);
            unsigned char * pixels = &(target_pixels[x * 4 + y * (width * 4)]);
            if (hit) {
                // TODO Write your shader here.
                // TODO rename fid to something else
                unsigned int fid = isect.prim_id;
                const ca::Vec3f& v_normal = squares.normals[fid];

                float red_color = 0.0f;

                // global directional light
                {
                    static const ca::Vec3f negZ = {0.0f, 0.0f, -1.0f};
                    const float f_dot = ca::dot(v_normal, negZ);
                    if (f_dot >= 0.0f) {
                        red_color += f_dot * 100 + 5.0f;
                    }
                }

                // global sphere light
                {
                    static const ca::Vec3f v_sphereLight = {0.0f,0.0f,0.0f};
                    const ca::Vec3u &v_face = squares.faces[fid];
                    const ca::Vec3f &v_p1 = squares.verts[v_face.x];
                    const ca::Vec3f &v_p2 = squares.verts[v_face.y];
                    const ca::Vec3f &v_p3 = squares.verts[v_face.z];
                    const ca::Vec3f v_u = v_p2 - v_p1;
                    const ca::Vec3f v_v = v_p3 - v_p1;
                    const ca::Vec3f v_hit = v_u * isect.u + v_v * isect.v + v_p1;
                    const ca::Vec3f v_toLight = v_hit - v_sphereLight;
                    const float f_dot = ca::dot(v_toLight, v_normal);
                    if (f_dot < 0.0f) {
                        float mult = 5.0f / ca::length(v_toLight);
                        if (mult >= 1.0f) {
                            mult = 1.0f;
                        }
                        red_color += mult * 100;
                    }
                }
                // pixels[0] = (normal.x * 0.5f + 0.5f) * 240;
                // pixels[1] = (normal.y * 0.5f + 0.5f) * 240;
                // pixels[2] = (normal.z * 0.5f + 0.5f) * 240;
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

void drawInvertedCube(
    RenderObject &cube,
    const ca::Vec3f& pos,
    const ca::Mat3f& rot_mat,
    const float scale = 10.0f)
{
    const float extent = scale * 0.5f;
    const ca::Vec3f cube_verts [8] = {
        {-extent + pos.x, -extent + pos.y, -extent + pos.z},
        {-extent + pos.x, -extent + pos.y,  extent + pos.z},
        {-extent + pos.x,  extent + pos.y, -extent + pos.z},
        {-extent + pos.x,  extent + pos.y,  extent + pos.z},
        { extent + pos.x, -extent + pos.y, -extent + pos.z},
        { extent + pos.x, -extent + pos.y,  extent + pos.z},
        { extent + pos.x,  extent + pos.y, -extent + pos.z},
        { extent + pos.x,  extent + pos.y,  extent + pos.z}
    };
    ca::Vec3f cube_normals [12];
    const unsigned length_v = (unsigned)(cube.verts.size());
    const ca::Vec3u cube_faces [12] = {
        {length_v + 1, length_v + 7, length_v + 5},
        {length_v + 7, length_v + 1, length_v + 3},
        {length_v + 5, length_v + 6, length_v + 4},
        {length_v + 6, length_v + 5, length_v + 7},
        {length_v + 3, length_v + 6, length_v + 7},
        {length_v + 6, length_v + 3, length_v + 2},
        {length_v + 4, length_v + 1, length_v + 5},
        {length_v + 1, length_v + 4, length_v + 0},
        {length_v + 0, length_v + 3, length_v + 1},
        {length_v + 3, length_v + 0, length_v + 2},
        {length_v + 6, length_v + 0, length_v + 4},
        {length_v + 0, length_v + 6, length_v + 2}
    };

    for (size_t i = 0; i < 12; i++) {
        const ca::Vec3f& p1 = cube_verts[cube_faces[i].x - length_v];
        const ca::Vec3f& p2 = cube_verts[cube_faces[i].y - length_v];
        const ca::Vec3f& p3 = cube_verts[cube_faces[i].z - length_v];
        ca::Vec3f u = p2 - p1;
        ca::Vec3f v = p3 - p1;

        ca::Vec3f normal = {
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x
        };
        normal = rot_mat * normal;
        ca::normalize_modify(normal);
        cube_normals[i] = normal;
    }
    size_t cube_verts_i = cube.verts.size();
    cube.verts.insert(cube.verts.end(), cube_verts, cube_verts + 8 );
    cube.faces.insert(cube.faces.end(), cube_faces, cube_faces + 12);
    cube.normals.insert(cube.normals.end(), cube_normals, cube_normals + 12);

    // rotate the vertices
    while (cube_verts_i < cube.verts.size()) {
        cube.verts[cube_verts_i] = (rot_mat * cube.verts[cube_verts_i]);
        cube_verts_i++;
    }
}

void drawCube(
    RenderObject &cube,
    const ca::Vec3f& pos,
    const ca::Mat3f& rot_mat,
    const float scale = 1.0f)
{
    const float extent = scale * 0.5f;
    const ca::Vec3f cube_verts [8] = {
        {-extent + pos.x, -extent + pos.y, -extent + pos.z},
        {-extent + pos.x, -extent + pos.y,  extent + pos.z},
        {-extent + pos.x,  extent + pos.y, -extent + pos.z},
        {-extent + pos.x,  extent + pos.y,  extent + pos.z},
        { extent + pos.x, -extent + pos.y, -extent + pos.z},
        { extent + pos.x, -extent + pos.y,  extent + pos.z},
        { extent + pos.x,  extent + pos.y, -extent + pos.z},
        { extent + pos.x,  extent + pos.y,  extent + pos.z}
    };
    ca::Vec3f cube_normals [12];
    const unsigned length_v = (unsigned)(cube.verts.size());
    const ca::Vec3u cube_faces [12] = {
        {length_v + 5, length_v + 7, length_v + 1},
        {length_v + 3, length_v + 1, length_v + 7},
        {length_v + 4, length_v + 6, length_v + 5},
        {length_v + 7, length_v + 5, length_v + 6},
        {length_v + 7, length_v + 6, length_v + 3},
        {length_v + 2, length_v + 3, length_v + 6},
        {length_v + 5, length_v + 1, length_v + 4},
        {length_v + 0, length_v + 4, length_v + 1},
        {length_v + 1, length_v + 3, length_v + 0},
        {length_v + 2, length_v + 0, length_v + 3},
        {length_v + 4, length_v + 0, length_v + 6},
        {length_v + 2, length_v + 6, length_v + 0}
    };

    for (size_t i = 0; i < 12; i++) {
        const ca::Vec3f& p1 = cube_verts[cube_faces[i].x - length_v];
        const ca::Vec3f& p2 = cube_verts[cube_faces[i].y - length_v];
        const ca::Vec3f& p3 = cube_verts[cube_faces[i].z - length_v];
        ca::Vec3f u = p2 - p1;
        ca::Vec3f v = p3 - p1;

        ca::Vec3f normal = {
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x
        };
        normal = rot_mat * normal;
        ca::normalize_modify(normal);
        cube_normals[i] = normal;
    }
    size_t cube_verts_i = cube.verts.size();
    cube.verts.insert(cube.verts.end(), cube_verts, cube_verts + 8 );
    cube.faces.insert(cube.faces.end(), cube_faces, cube_faces + 12);
    cube.normals.insert(cube.normals.end(), cube_normals, cube_normals + 12);

    // rotate the vertices
    while (cube_verts_i < cube.verts.size()) {
        cube.verts[cube_verts_i] = (rot_mat * cube.verts[cube_verts_i]);
        cube_verts_i++;
    }
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
    unsigned num_copies = 1;
    // if (myFile.is_open()) {
    //     myFile >> numVerts >> numFaces;
    //     bunny.verts.resize(numVerts * num_copies);
    //     bunny.normals.resize(numFaces * num_copies);
    //     bunny.faces.resize(numFaces * num_copies);
    //     for (unsigned i = 0; i < numVerts; i++) {
    //         char v;
    //         float v1, v2, v3;
    //         myFile >> v >> v1 >> v2 >> v3;
    //         {
    //             ca::Vec3f pos = {v1, v2, v3};
    //             drawCube(squares,pos);
    //         }
    //         for (int j = 0; j < num_copies; j++) {
    //             ca::Vec3f pos = {v1 + j * 2, v2, v3};
    //             bunny.verts[i + j * numVerts] = pos;
    //         }
    //     }
    //     for (unsigned i = 0; i < numFaces; i++) {
    //         char f;
    //         int f1, f2, f3;
    //         myFile >> f >> f1 >> f2 >> f3;

    //         // TODO switch to use ca::Vec3f
    //         ca::Vec3f& p1 = bunny.verts[f1];
    //         ca::Vec3f& p2 = bunny.verts[f2];
    //         ca::Vec3f& p3 = bunny.verts[f3];
    //         ca::Vec3f u = p2 - p1;
    //         ca::Vec3f v = p3 - p1;

    //         ca::Vec3f normal = {
    //             u.y * v.z - u.z * v.y,
    //             u.z * v.x - u.x * v.z,
    //             u.x * v.y - u.y * v.x
    //         };
    //         ca::normalize_modify(normal);

    //         for (int j = 0; j < num_copies; j++) {
    //             ca::Vec3u face = {
    //                 f1 + j * numVerts,
    //                 f2 + j * numVerts,
    //                 f3 + j * numVerts
    //             };
    //             bunny.faces[i + j * numFaces] = face;
    //             bunny.normals[i + j * numFaces] = normal;
    //         }
    //     }
    // } else {
    //     // TODO exit
    // }

    // initialize global values
    forward = {0.0f, 0.0f, 1.0f};
    right   = {1.0f, 0.0f, 0.0f};
    look_matrix = ca::Mat3f::Identity();
    update_look_matrix(0.f, 0.f);

    ca::Quat q_rotate = ca::axis_angle_quat({1.0f,0.0f,0.0f}, -1.0f);

    drawInvertedCube(squares, ca::Vec3f{0.f,0.f,0.f}, ca::Mat3f::Identity());

    drawCube(squares, ca::Vec3f{0.f,0.f,0.f}, ca::RotationMat3f(q_rotate));
    drawCube(squares, ca::Vec3f{-1.5f,0.0f,0.0f}, ca::RotationMat3f(q_rotate));
    drawCube(squares, ca::Vec3f{ 1.5f,0.0f,0.0f}, ca::RotationMat3f(q_rotate));

    nanort::BVHBuildOptions<float> options;
    NanortRenderData bunny_render_data = build_scene(bunny, options);
    NanortRenderData squares_render_data = build_scene(squares, options);
    // Initialize SDL

    SDLWindowSurfacePair sdl_init_result = SDL_init_window();
    SDL_Window * mainWindow = sdl_init_result.mainWindow;
    SDL_Surface * screenSurface = sdl_init_result.screenSurface;

    // SDL loop
    {
        SDL_Surface * renderedSurface = SDL_rendered_surface_init();
        render_scene(width, height, squares_render_data, renderedSurface);

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
                                update_look_matrix( 0.0f,  0.1f);
                                break;

                            case SDLK_RIGHT:
                                update_look_matrix( 0.0f, -0.1f);
                                break;
                            case SDLK_UP:
                                update_look_matrix(-0.1f,  0.0f);
                                break;

                            case SDLK_DOWN:
                                update_look_matrix( 0.1f,  0.0f);
                                break;
                        }
                    }
                    else if( e.type == SDL_QUIT )
                    {
                        quit = true;
                        continue;
                    }
                } while ( SDL_PollEvent( &e ) != 0 );
                render_scene(width, height, squares_render_data, renderedSurface);
                if (SDL_BlitScaled( renderedSurface, NULL, screenSurface, NULL )) {
                    printf("ERROR>>> %s\n", SDL_GetError());
                }
            }
            SDL_UpdateWindowSurface(mainWindow);
        }
    }

    return 0;
}
