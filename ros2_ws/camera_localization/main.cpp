#include <cstdio>
#include "ray_ground_intersection.hpp"

int main()
{
    // ------------------------------------------------------------------
    // Camera intrinsics + distortion  (set once, reuse forever)
    // ------------------------------------------------------------------
    CameraParams cam;
    cam.fx = 499.59041324;
    cam.fy = 498.40102851;
    cam.cx = 320.11657344;
    cam.cy = 245.76315875;

    DistortionCoeffs dist;
    dist.k1 =  0.18333092;
    dist.k2 = -0.69846172;
    dist.p1 = -0.00170541;
    dist.p2 = -0.00092574;
    dist.k3 =  0.68747843;

    // ------------------------------------------------------------------
    // Create projector once.  In ROS2 node: create in constructor.
    // ------------------------------------------------------------------
    GroundProjector projector(cam, dist);

    // ------------------------------------------------------------------
    // Each frame: update pose, then project pixels.
    // In ROS2 node: call setPose() in your IMU/odometry callback,
    //               call project() in your detection callback.
    // ------------------------------------------------------------------
    projector.setPose(
        /*x=*/0.0, /*y=*/0.0, /*z=*/0.835,   // ENU position (m)
        /*yaw=*/  deg2rad(0.0),               // 0 = North
        /*pitch=*/deg2rad(0.0),              // 90 = straight down
        /*roll=*/ deg2rad(0.0)
    );

    // Distorted pixels from screenshot (e.g. bounding box bottom centers)
    std::vector<Pixel> pixels = {
        {(int)cam.cx, (int)cam.cy},  // principal point -> directly below
        {514, 362},
        {320, 416},
        {510, 403},
    };

    auto results = projector.project(pixels);

    // ------------------------------------------------------------------
    // Print
    // ------------------------------------------------------------------
    printf("%-10s %-10s %-14s %-14s %-12s\n",
           "u(col)", "v(row)", "East (m)", "North (m)", "dist (m)");
    printf("%s\n", std::string(62,'-').c_str());
    for (auto& r : results) {
        if (r.valid)
            printf("%-10d %-10d %-14.4f %-14.4f %-12.4f\n",
                   r.u, r.v, r.world_x, r.world_y, r.distance);
        else
            printf("%-10d %-10d  NO HIT: %s\n", r.u, r.v, r.reason.c_str());
    }

    return 0;
}