/*
 Resolution: 320 x 480
 Format: VG_LITE_RGB565
 Transformation: Rotate/Scale/Translate
 Alpha Blending: None
 Related APIs: vg_lite_clear/vg_lite_translate/vg_lite_scale/vg_lite_rotate/vg_lite_blit
 Description: Load outside landscape image, then blit it to blue dest buffer with rotate/scale/translate.
 Image filter type is selected by hardware feature gcFEATURE_BIT_VG_IM_FILTER(ON: VG_LITE_FILTER_BI_LINEAR, OFF: VG_LITE_FILTER_POINT).
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "vg_lite.h"
#include "vg_lite_ex.h"
#include "vg_lite_util.h"
#include "tc_vglite.h"
#include "board.h"


/*Porting notes
* 1)define RESX, RESY, FRAME_BUFFERS, IMG_SIZE, IMAGE_COUNT
* 2)Modify "load_mult_objects" function (frame-buffer format, stride, input images path)
*/

#define COUNT_FPS
// #define MODE_INTERRACTIVE
#ifndef RESX
    #define RESX 320
#endif

#ifndef RESY
    #define RESY 272
#endif

#ifndef DEBUG_OVERDRAWS
    #define DEBUG_OVERDRAWS 0
#endif

#define FRAME_BUFFERS 1
#define IMG_SIZE      128
#define IMAGE_COUNT   10
#define M_PI          3.14159265358979323846

typedef struct _quad_t
{
    float x0, y0, z0, w0;
    float x1, y1, z1, w1;
    float x2, y2, z2, w2;
    float x3, y3, z3, w3;
} quad_t;

static int   fb_width = 320, fb_height = 272;
static float fb_scale = 1.0f;

static vg_lite_buffer_t buffer;     //offscreen framebuffer object for rendering.
static vg_lite_buffer_t *fb;
static vg_lite_buffer_t imgs_rgba[IMAGE_COUNT];

static float w0;
static float w1;
static float w2 = 1;
static float tx;
static float ty;

#define IMG_WIDTH   (128)
#define IMG_HEIGHT  (128)

ALIGN(64)
const static uint8_t img_data[IMAGE_COUNT][IMG_SIZE * IMG_SIZE * 3] =
{
    {
#include "im01_128.dat"
    },
    {
#include "im02_128.dat"
    },
    {
#include "im03_128.dat"
    },
    {
#include "im04_128.dat"
    },
    {
#include "im05_128.dat"
    },
    {
#include "im06_128.dat"
    },
    {
#include "im07_128.dat"
    },
    {
#include "im08_128.dat"
    },
    {
#include "im09_128.dat"
    },
    {
#include "im10_128.dat"
    },
};

static void cleanup(void)
{
    if (buffer.handle != NULL)
    {
        // Free the buffer memory.
        vg_lite_free(&buffer);
    }

    vg_lite_close();
}

#if 0
void print_vertex(const char *s, vg_lite_vertex_t *vertex)
{
    rt_kprintf("%s:%10.3f,%10.3f,%10.3f\n", s, vertex->x, vertex->y, vertex->z);
}

void print_point(const char *s, vg_lite_vertex_t *vertex)
{
    rt_kprintf("%s:%10.3f,%10.3f\n", s, vertex->x, vertex->y);
}

static void print_matrix_4x4(const char *s, vg_lite_matrix_4x4_t *mat)
{
    rt_kprintf("%s\n", s);
    rt_kprintf("%10.3f  %10.3f  %10.3f  %10.3f\n", mat->m[0][0], mat->m[0][1], mat->m[0][2], mat->m[0][3]);
    rt_kprintf("%10.3f  %10.3f  %10.3f  %10.3f\n", mat->m[1][0], mat->m[1][1], mat->m[1][2], mat->m[0][3]);
    rt_kprintf("%10.3f  %10.3f  %10.3f  %10.3f\n", mat->m[2][0], mat->m[2][1], mat->m[2][2], mat->m[0][3]);
    rt_kprintf("%10.3f  %10.3f  %10.3f  %10.3f\n", mat->m[3][0], mat->m[3][1], mat->m[3][2], mat->m[3][3]);
}

void print_matrix(const char *s, vg_lite_matrix_t *mat)
{
    rt_kprintf("%s\n", s);
    rt_kprintf("%10.3f,%10.3f,%10.3f\n", mat->m[0][0], mat->m[0][1], mat->m[0][2]);
    rt_kprintf("%10.3f,%10.3f,%10.3f\n", mat->m[1][0], mat->m[1][1], mat->m[1][2]);
    rt_kprintf("%10.3f,%10.3f,%10.3f\n", mat->m[2][0], mat->m[2][1], mat->m[2][2]);
}

#else
#define print_vertex(s, vertex)
#define print_point(s, vertex)
#define print_matrix_4x4(s, mat)
#define print_matrix(s, mat)

#endif

float tc_ez_linear(float A, float B, float steps, float cur_step)
{
    return (B - A) / steps * cur_step + A;
}

static float tc_cos(float angle_deg)
{
    return cosf(angle_deg / 180.f * M_PI);
}

static float tc_sin(float angle_deg)
{
    return sinf(angle_deg / 180.f * M_PI);
}

static void
load_mult_objects(void)
{
    int i = 0;

    for (i = 0; i < IMAGE_COUNT; ++i)
    {
        vg_lite_init_buf(&imgs_rgba[i], IMG_WIDTH, IMG_HEIGHT, VG_LITE_BGRA5658, (void *)HCPU_MPI_SBUS_ADDR((uint32_t)&img_data[i][0]));
    }
}

static float FoV    = 30.f;
static float y_orig = 0;
static float x_orig = 0;

const float img_width     = 0.2f;
const float img_angle_deg = 90.f;
const float back_rail_z   = 0.98f;
const float front_img_z   = 0.7f;

static float front_rail_z;
static float shift_par0, shift_par3;
static float r_big, r_small;
static float centre_right_x, img_x_proj;


//Transform coordinates from [-1,1] to [0, RESX] and [0, RESY]
vg_lite_matrix_4x4_t proj_matrix;
static void transform_quad_coords(vg_lite_quad_t quad)
{

    vg_lite_vertex_t v0, v1, v2, v3;
    float w0;

    v0 = quad[0];
    v1 = quad[1];
    v2 = quad[2];
    v3 = quad[3];
    w0 = 1;
    vg_lite_4x4_obj_to_ndc_coords(&proj_matrix, 0, 0, fb->width, fb->height, 1.f, 100.0f, &v0.x, &v0.y, &v0.z, &w0);
    vg_lite_4x4_obj_to_ndc_coords(&proj_matrix, 0, 0, fb->width, fb->height, 1.f, 100.0f, &v1.x, &v1.y, &v1.z, &w0);
    vg_lite_4x4_obj_to_ndc_coords(&proj_matrix, 0, 0, fb->width, fb->height, 1.f, 100.0f, &v2.x, &v2.y, &v2.z, &w0);
    vg_lite_4x4_obj_to_ndc_coords(&proj_matrix, 0, 0, fb->width, fb->height, 1.f, 100.0f, &v3.x, &v3.y, &v3.z, &w0);

    print_vertex("v0_ndc", &v0);
    print_vertex("v1_ndc", &v1);
    print_vertex("v2_ndc", &v2);
    print_vertex("v3_ndc", &v3);

    vg_lite_4x4_ndc_to_win_coords(0, 0, fb->width, fb->height, &v0.x, &v0.y);
    vg_lite_4x4_ndc_to_win_coords(0, 0, fb->width, fb->height, &v1.x, &v1.y);
    vg_lite_4x4_ndc_to_win_coords(0, 0, fb->width, fb->height, &v2.x, &v2.y);
    vg_lite_4x4_ndc_to_win_coords(0, 0, fb->width, fb->height, &v3.x, &v3.y);

    print_vertex("v0_screen", &v0);
    print_vertex("v1_screen", &v1);
    print_vertex("v2_screen", &v2);
    print_vertex("v3_screen", &v3);

    quad[0] = v0;
    quad[1] = v1;
    quad[2] = v2;
    quad[3] = v3;
}


static void calc_circle(float *x_off, float *z_off, float angle, float r)
{
    *x_off = tc_cos(angle) * r;
    *z_off = tc_sin(angle) * r;
}

static void cover_flow_init(void)
{
    vg_lite_perspective_4x4(FoV, (float)RESX / RESY, 1, 100, &proj_matrix);

    print_matrix_4x4("proj_4x4 matrix", &proj_matrix);

    img_x_proj = tc_cos(img_angle_deg) * img_width;
    float img_z_proj = tc_sin(img_angle_deg) * img_width;

    front_rail_z = back_rail_z - img_z_proj;

    r_big   = (back_rail_z  - front_img_z) / tc_sin(img_angle_deg);
    r_small = (front_rail_z - front_img_z) / tc_sin(img_angle_deg);

    float x0_front_x = -img_width / 2;

    centre_right_x = x0_front_x + r_big;

    float right_critical_x0 = centre_right_x - tc_cos(img_angle_deg) * r_big;

    shift_par0 = -(right_critical_x0 + img_x_proj - 1) * 0.5f;
    shift_par3 = 1.f - shift_par0;

#if 0
    float centre_right_z = front_img_z;

    float centre_left_x  = x0_front_x - r_small;
    float centre_left_z  = front_img_z;
    float right_critical_z0 = back_rail_z;
    float right_critical_x1 = centre_right_x - tc_cos(img_angle_deg) * r_small;
    float right_critical_z1 = front_rail_z;

    float x0_front_z = front_img_z;

    float x1_front_x = img_width / 2;
    float x1_front_z = front_img_z;
    printf("%f\n", img_width);
    printf("%f\n", img_angle_deg);
    printf("%f\n", img_x_proj);
    printf("%f\n", img_z_proj);
    printf("%f\n", back_rail_z);
    printf("%f\n", front_img_z);
    printf("%f\n", front_rail_z);
    printf("%f\n", r_big);
    printf("%f\n", r_small);

    printf("\n");

    printf("%f\t%f\n", x0_front_x, x0_front_z);
    printf("%f\t%f\n", x1_front_x, x1_front_z);
    printf("%f\t%f\n", centre_right_x, centre_right_z);
    printf("%f\t%f\n", centre_left_x, centre_left_z);
    printf("%f\t%f\n", right_critical_x0, right_critical_z0);
    printf("%f\t%f\n", right_critical_x1, right_critical_z1);

    printf("\n");

    printf("%f\n", shift_par0);
    printf("%f\n", shift_par3);
#endif
}

static void map_quad_points(quad_t *q, float shift_par)
{
    if (shift_par < shift_par0)
    {
        //0 - 0.3
        q->x0 = -2 * shift_par + 1 - img_x_proj;
        q->z0 = back_rail_z;

        q->x1 = q->x0 + img_x_proj;
        q->z1 = front_rail_z;
    }
    else if (shift_par < 0.5f)
    {
        //shift_par0 - 0.5
        //angle 71.6 - 0
        float angle = tc_ez_linear(img_angle_deg, 0.f, 0.5f - shift_par0, shift_par - shift_par0);
        calc_circle(&q->x0, &q->z0, angle, r_big);

        q->x0 = centre_right_x - q->x0;
        q->z0 = front_img_z + q->z0;

        calc_circle(&q->x1, &q->z1, angle, r_small);

        q->x1 = centre_right_x - q->x1;
        q->z1 = front_img_z + q->z1;
    }
    else if (shift_par < shift_par3)
    {
        //0.5 - shift_par3
        //angle 0 - 71.6
        float angle = tc_ez_linear(0.f, img_angle_deg, shift_par3 - 0.5f, shift_par - 0.5f);
        calc_circle(&q->x0, &q->z0, angle, r_small);

        q->x0 = -centre_right_x + q->x0;
        q->z0 = front_img_z + q->z0;

        calc_circle(&q->x1, &q->z1, angle, r_big);

        q->x1 = -centre_right_x + q->x1;
        q->z1 = front_img_z + q->z1;
    }
    else
    {
        //0.7 - 1.0
        q->x0 = -2 * shift_par + 1;
        q->z0 = front_rail_z;


        q->x1 = q->x0 + img_x_proj;
        q->z1 = back_rail_z;
    }


    ///X
    q->x2 = q->x1;
    q->x3 = q->x0;

    ///Y
    q->y0 = q->y1 = -0.5f * img_width;
    q->y2 = q->y3 =  0.5f * img_width;

    ///Z
    q->z2 = q->z1 = -q->z1;
    q->z3 = q->z0 = -q->z0;

    ///W
    q->w0 = q->w1 = q->w2 = q->w3 = 1.f;
}


static float map_to_ratio(float val, float break_point, float laska)
{
    float ret_val = 0.f;

    if (val <= break_point)
    {
        return val / (1.f + laska);
    }

    ret_val += break_point;
    val     -= break_point;

    float before_laska = (0.5f - break_point) * 2.f;
    float after_laska  = (0.5f - break_point) * 2.f + laska;

    if (val < before_laska)
    {
        ret_val += val * after_laska / before_laska;
        return ret_val / (1.f + laska);
    }

    ret_val += after_laska;

    val     -= before_laska;
    ret_val += val;

    return ret_val / (1.f + laska);
}

static void draw_texture(vg_lite_buffer_t *img, vg_lite_quad_t quad, int alpha)
{
    vg_lite_matrix_t matrix;
    vg_lite_matrix_t proj;
    vg_lite_error_t err;
    vg_lite_filter_t filter;
    vg_lite_vertex_t p0, p1, p2, p3;
    vg_lite_vertex_t normal;
    int r;

    filter = VG_LITE_FILTER_BI_LINEAR;

    err = vg_lite_get_texture_map_matrix(FoV, IMG_WIDTH, IMG_HEIGHT, fb->width, fb->height,
                                         quad, &proj);

    if (VG_LITE_SUCCESS != err)
    {
        rt_kprintf("calc perspective matrix fail\n");
        return;
    }

    p0.x = 0;
    p0.y = 0;
    p1.x = 0;
    p1.y = IMG_HEIGHT;
    p2.x = IMG_WIDTH;
    p2.y = IMG_HEIGHT;
    p3.x = IMG_WIDTH;
    p3.y = 0;
    vg_lite_mat_vertex_multiply(&proj, &p0.x, &p0.y);
    vg_lite_mat_vertex_multiply(&proj, &p1.x, &p1.y);
    vg_lite_mat_vertex_multiply(&proj, &p2.x, &p2.y);
    vg_lite_mat_vertex_multiply(&proj, &p3.x, &p3.y);
    print_point("p0", &p0);
    print_point("p1", &p1);
    print_point("p2", &p2);
    print_point("p3", &p3);


    // Blit the image using the matrix.
    err = vg_lite_blit(fb, img, &proj, VG_LITE_BLEND_DST_OVER, 0, filter);
    if (VG_LITE_SUCCESS == err)
    {
        vg_lite_finish();
    }
    else
    {
        rt_kprintf("blit error:%d\n", err);
    }
}

int cover_flow()
{
//**************    INIT    *************************************

    load_mult_objects();
    cover_flow_init();

//****************************************************************
//**************    Background    ********************************


//****************************************************************
//**************** Coverflow *************************************

    float shift_par;
#ifdef COUNT_FPS
    float sec0 = rt_tick_get();
#endif

    int frames  = 0;
    float secs  = 0.5; //TODO =1.0
    int cur_img = 0;
    float sec_init     = 0.f;
    float cur_sec      = 0.f;

    while (1)
    {
        // Clear the buffer with white.
        vg_lite_clear(fb, NULL, 0x40404040);
        // vg_lite_finish();

        //STOP_COUNT: number of images inside frustum
#define STOP_COUNT (40)

        const float break_point = 0.5f - 1.f / STOP_COUNT;
        float clip_left  = RESX;
        float clip_right = 0;

        for (int  stop = STOP_COUNT - 1; stop >= 0; --stop)
        {
            int stop_idx = (stop & 0x1) ? STOP_COUNT - ((stop + 1) >> 1) : stop >> 1 ;
            int img_idx  = (cur_img + stop_idx + IMAGE_COUNT * 10 - (STOP_COUNT >> 1)) % IMAGE_COUNT;

            // rt_kprintf("idx:%d,%d,%d\n", stop, stop_idx, img_idx);
            // rt_kprintf("clip:%f,%f\n", clip_right, clip_left);

            if (stop_idx > STOP_COUNT >> 1)
            {
                if (clip_right >= RESX) continue;
                //right side pictures
            }
            else     //if ( stop_idx > STOP_COUNT>>1)
            {
                if (clip_left < 0) continue;
                //left side pictures
            }

            float shift_target, shift_prev;

            shift_target = tc_ez_linear(.5f, 0.f, (STOP_COUNT >> 1), stop_idx) + 0.5f;
            shift_prev   = tc_ez_linear(.5f, 0.f, STOP_COUNT >> 1, stop_idx + 1) + 0.5f;

            shift_target = map_to_ratio(shift_target, break_point, 0.19f);

            shift_prev   = map_to_ratio(shift_prev, break_point, 0.19f);

            shift_par    = tc_ez_linear(shift_prev, shift_target, 1.0, cur_sec);

            quad_t q1;
            vg_lite_quad_t vertices;
            map_quad_points(&q1, shift_par);

#if 0
            transform_quad_coords(&q1);


            //make sides of quads integers
            //so that clipping is done right
            q1.x0 = q1.x3 = (int) q1.x0;
            q1.x1 = q1.x2 = (int) q1.x1;

            ///Reflections
            bind_img(&imgs_rgba[(img_idx) % IMAGE_COUNT]);
#endif

            float screen_img_height_0 = q1.y3 - q1.y0;
            float screen_img_height_1 = q1.y2 - q1.y1;
            vg_lite_matrix_4x4_t proj_4x4;

            vertices[0].x = q1.x3;
            vertices[0].y = q1.y3 - 2.f * screen_img_height_0;
            vertices[0].z = q1.z3;
            vertices[1].x = q1.x0;
            vertices[1].y = q1.y0;
            vertices[1].z = q1.z0;
            vertices[2].x = q1.x1;
            vertices[2].y = q1.y1;
            vertices[2].z = q1.z1;
            vertices[3].x = q1.x2;
            vertices[3].y = q1.y2 - 2.f * screen_img_height_1;
            vertices[3].z = q1.z2;

#if 1
            // rt_kprintf("mirror image:%d\n", (img_idx)%IMAGE_COUNT);
            print_vertex("v0", &vertices[0]);
            print_vertex("v1", &vertices[1]);
            print_vertex("v2", &vertices[2]);
            print_vertex("v3", &vertices[3]);
            vg_lite_set_dest_global_alpha(VG_LITE_GLOBAL, 180);
            draw_texture(&imgs_rgba[(img_idx) % IMAGE_COUNT], vertices, 255);
#endif

            vertices[0].x = q1.x3;
            vertices[0].y = q1.y3;
            vertices[0].z = q1.z3;
            vertices[1].x = q1.x0;
            vertices[1].y = q1.y0;
            vertices[1].z = q1.z0;
            vertices[2].x = q1.x1;
            vertices[2].y = q1.y1;
            vertices[2].z = q1.z1;
            vertices[3].x = q1.x2;
            vertices[3].y = q1.y2;
            vertices[3].z = q1.z2;

            // rt_kprintf("normal image\n");
            print_vertex("v0", &vertices[0]);
            print_vertex("v1", &vertices[1]);
            print_vertex("v2", &vertices[2]);
            print_vertex("v3", &vertices[3]);
            vg_lite_set_dest_global_alpha(VG_LITE_GLOBAL, 0);
            draw_texture(&imgs_rgba[(img_idx) % IMAGE_COUNT], vertices, 255);
            transform_quad_coords(vertices);

#if 1
            if (stop_idx > STOP_COUNT >> 1)
            {
                //right side pictures
                clip_right = vertices[2].x;
            }
            else if (stop_idx < STOP_COUNT >> 1)
            {
                //left side pictures
                clip_left = vertices[0].x;
            }
            else
            {
                clip_left  = vertices[0].x;
                clip_right = vertices[2].x;
            }

            if (clip_right >= RESX && clip_left < 0)
            {
                break;
            }
#endif
        }

#ifdef COUNT_FPS
        ++frames;
        if (frames % 100 == 0)
        {
            float sec1 = rt_tick_get();
            rt_kprintf("fps: %.2f\n", (100.f / ((sec1 - sec0) / 1000.0f)));
            sec0 = rt_tick_get();
        }
#endif

        tc_vg_send_data_to_lcd(fb->memory, fb->width, fb->height, RTGRAPHIC_PIXEL_FORMAT_RGB565);
        // rt_thread_mdelay(300000000);

        //Small cur_sec => slow transition (does NOT affect the actual fps)
        cur_sec += 0.03;

        if (cur_sec > secs + 0.5)
        {
            cur_sec  -= secs + 0.5;
            sec_init += secs + 0.5;
            cur_img   = (cur_img + 1) % IMAGE_COUNT;
        }
        else if (cur_sec < 0.f)
        {
            cur_sec  += secs + 0.5;
            sec_init -= secs + 0.5;
            cur_img   = (cur_img + IMAGE_COUNT - 1) % IMAGE_COUNT;
        }

    }
    // #endif
    return 0;
}


static void tc_vglite_coverflow(int argc, char **argv)
{
    // Initialize vg_lite engine.
    vg_lite_error_t error = VG_LITE_SUCCESS;
    TC_VGLITE_CHECK_ERROR(vg_lite_set_command_buffer_size(16 << 10));
    TC_VGLITE_CHECK_ERROR(vg_lite_init(fb_width, fb_height));

    // Set image filter type according to hardware feature.

    printf("Framebuffer size: %d x %d\n", fb_width, fb_height);

    // Allocate the off-screen buffer.
    buffer.width  = fb_width;
    buffer.height = fb_height;
    buffer.format = VG_LITE_BGR565;

    TC_VGLITE_CHECK_ERROR(vg_lite_allocate(&buffer));
    fb = &buffer;

    cover_flow();

ErrorHandler:
    // Cleanup.
    cleanup();
}

UTEST_TC_EXPORT(tc_vglite_coverflow, "tc_vglite_1_2", tc_vglite_init, tc_vglite_cleanup, 10);


