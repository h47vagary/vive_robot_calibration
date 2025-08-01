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

bool vive_get_pose_quaternion(double* x, double* y, double* z,
                   double* qx, double* qy, double* qz, double* qw,
                   uint64_t* button_mask)
{
    if (!x || !y || !z || !qx || !qy || !qz || !qw || !button_mask)
        return false;

    return g_tracker.get_pose(*x, *y, *z, *qx, *qy, *qz, *qw, *button_mask);
}

bool vive_get_pose_euler(double *x, double *y, double *z, double *A, double *B, double *C, uint64_t *button_mask)
{
    if (!x || !y || !z || !A || !B || !C || !button_mask)
        return false;

    return g_tracker.get_pose(*x, *y, *z, *A, *B, *C, *button_mask);
}

bool vive_get_pose_quaternion_non_blocking(double *x, double *y, double *z, double *qx, double *qy, double *qz, double *qw, uint64_t *button_mask)
{
    if (!x || !y || !z || !qx || !qy || !qz || !qw || !button_mask)
        return false;

    return g_tracker.get_pose_non_blocking(*x, *y, *z, *qx, *qy, *qz, *qw, *button_mask);
}

bool vive_get_pose_euler_non_blocking(double *x, double *y, double *z, double *A, double *B, double *C, uint64_t *button_mask)
{
    if (!x || !y || !z || !A || !B || !C || !button_mask)
        return false;

    return g_tracker.get_pose_non_blocking(*x, *y, *z, *A, *B, *C, *button_mask);
}
