#include "vive_wrapper.h"
#include "vive_tracker.h"

using namespace htc_vive;

static ViveTracker g_tracker;

bool vive_initialize()
{
    return g_tracker.initialize();
}

void vive_shutdown()
{
    g_tracker.shutdown();
}

bool vive_find_tracker()
{
    return g_tracker.find_tracker();
}

void vive_set_origin(float x, float y, float z,
                     float qx, float qy, float qz, float qw)
{
    g_tracker.set_origin(x, y, z, qx, qy, qz, qw);
}

bool vive_get_pose(float* x, float* y, float* z,
                   float* qx, float* qy, float* qz, float* qw,
                   uint64_t* button_mask)
{
    if (!x || !y || !z || !qx || !qy || !qz || !qw || !button_mask)
        return false;

    return g_tracker.get_pose(*x, *y, *z, *qx, *qy, *qz, *qw, *button_mask);
}

bool vive_get_pose_abc(double *x, double *y, double *z, double *A, double *B, double *C, uint64_t *button_mask)
{
    if (!x || !y || !z || !A || !B || !C || !button_mask)
        return false;

    return g_tracker.get_pose(*x, *y, *z, *A, *B, *C, *button_mask);
}

bool vive_get_relative_pose(float* x, float* y, float* z,
                            float* qx, float* qy, float* qz, float* qw,
                            uint64_t* button_mask)
{
    if (!x || !y || !z || !qx || !qy || !qz || !qw || !button_mask)
        return false;

    return g_tracker.get_relative_pose(*x, *y, *z, *qx, *qy, *qz, *qw, *button_mask);
}
