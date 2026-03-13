#pragma once
/**
 * ray_ground_intersection.hpp
 *
 * Ray-plane intersection for ground-plane localisation.
 *
 * ============================================================
 * COORDINATE CONVENTIONS
 * ============================================================
 *
 * World frame  :  ENU  (East-North-Up, right-handed)
 *                   +X = East,  +Y = North,  +Z = Up
 *                 Ground plane = Z = 0.
 *
 * Image frame  :  Origin top-left, u+ right, v+ down  (OpenCV standard)
 * Camera frame :  OpenCV convention  (X right, Y down, Z forward)
 *
 * ============================================================
 * ANGLE CONVENTIONS  (radians; use deg2rad() for degrees)
 * ============================================================
 *
 *  yaw   – compass heading, clockwise from North.
 *             0=North, 90=East, 180=South, 270=West
 *
 *  pitch – nose-down positive.
 *             0=horizontal, +90=straight down, -90=straight up
 *          Pass (mount_angle + drone_pitch) as a single value.
 *
 *  roll  – clockwise-from-behind positive.
 *             0=level, +30=right side down
 *
 * ============================================================
 * ROTATION
 * ============================================================
 *
 *  R_fix  : OpenCV cam axes → ENU body at zero attitude (cam faces North)
 *             East  (+X_enu) = +X_cam
 *             North (+Y_enu) = +Z_cam
 *             Up    (+Z_enu) = -Y_cam
 *
 *  R_att  : Rz(-yaw) * Rx(-pitch) * Ry(roll)
 *
 *  R_wc   : R_att * R_fix   (precomputed once per pose update)
 */

#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

// ---------------------------------------------------------------------------
// Vec3 / Mat3
// ---------------------------------------------------------------------------

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s)      const { return {x*s,   y*s,   z*s};   }
    double dot(const Vec3& o)     const { return x*o.x + y*o.y + z*o.z; }
    double norm()                 const { return std::sqrt(dot(*this));  }
    Vec3   normalized()           const { double n=norm(); return {x/n,y/n,z/n}; }
};

struct Mat3 {
    double m[3][3] = {};
    Vec3 operator*(const Vec3& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        };
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                for (int k=0;k<3;k++)
                    r.m[i][j] += m[i][k]*o.m[k][j];
        return r;
    }
};

inline Mat3 rotZ(double a) {
    double c=cos(a), s=sin(a);
    Mat3 R; R.m[0][0]=c; R.m[0][1]=-s;
            R.m[1][0]=s; R.m[1][1]= c; R.m[2][2]=1;
    return R;
}
inline Mat3 rotY(double a) {
    double c=cos(a), s=sin(a);
    Mat3 R; R.m[0][0]=c; R.m[0][2]=s; R.m[1][1]=1;
            R.m[2][0]=-s; R.m[2][2]=c;
    return R;
}
inline Mat3 rotX(double a) {
    double c=cos(a), s=sin(a);
    Mat3 R; R.m[0][0]=1;
            R.m[1][1]=c; R.m[1][2]=-s;
            R.m[2][1]=s; R.m[2][2]= c;
    return R;
}

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

struct CameraParams {
    // Intrinsics
    double fx = 1.0, fy = 1.0;
    double cx = 0.0, cy = 0.0;

    // Position in ENU
    double x = 0.0;   ///< East
    double y = 0.0;   ///< North
    double z = 0.0;   ///< Up (altitude above ground)

    // Orientation
    double yaw   = 0.0;  ///< compass CW from North (rad)
    double pitch = 0.0;  ///< nose-down positive (rad)
    double roll  = 0.0;  ///< CW-from-behind positive (rad)
};

struct DistortionCoeffs {
    double k1=0, k2=0, p1=0, p2=0, k3=0;  ///< OpenCV 5-parameter model
};

struct Pixel {
    int u = 0;  ///< column (0=left)
    int v = 0;  ///< row    (0=top)
};

struct IntersectionResult {
    int    u = 0, v = 0;
    bool   valid    = false;
    double world_x  = 0.0;   ///< East  (m)
    double world_y  = 0.0;   ///< North (m)
    double distance = 0.0;   ///< Euclidean camera→ground (m)
    std::string reason;      ///< set when !valid
};

// ---------------------------------------------------------------------------
// GroundProjector
// Instantiate once per camera. Call setPose() each frame with fresh IMU data.
// ---------------------------------------------------------------------------
class GroundProjector {
public:
    // ----------------------------------------------------------------
    // Construction: pass intrinsics + distortion once
    // ----------------------------------------------------------------
    GroundProjector(const CameraParams& cam, const DistortionCoeffs& dist)
        : cam_(cam), dist_(dist)
    {
        rebuildRotation();
    }

    // ----------------------------------------------------------------
    // Call this every time the drone pose changes (each IMU/EKF update)
    // Only recomputes the rotation matrix — cheap.
    // ----------------------------------------------------------------
    void setPose(double x, double y, double z,
                 double yaw, double pitch, double roll)
    {
        cam_.x = x; cam_.y = y; cam_.z = z;
        cam_.yaw = yaw; cam_.pitch = pitch; cam_.roll = roll;
        rebuildRotation();
    }

    // ----------------------------------------------------------------
    // Project a list of distorted pixels onto the ground plane (Z=0).
    // Undistortion is applied automatically.
    // ----------------------------------------------------------------
    std::vector<IntersectionResult>
    project(const std::vector<Pixel>& pixels) const
    {
        std::vector<IntersectionResult> results;
        results.reserve(pixels.size());

        const Vec3 origin = {cam_.x, cam_.y, cam_.z};

        for (const auto& px : pixels) {
            results.push_back(projectOne(px, origin));
        }
        return results;
    }

    // Single-pixel convenience
    IntersectionResult projectOne(const Pixel& px) const {
        return projectOne(px, {cam_.x, cam_.y, cam_.z});
    }

    // Expose R_wc for debug
    const Mat3& rotation() const { return R_wc_; }
    const CameraParams& params() const { return cam_; }

private:
    CameraParams    cam_;
    DistortionCoeffs dist_;
    Mat3            R_wc_;      // cached, rebuilt only on setPose()
    Vec3            R_wc_col2_; // third column of R_wc = direction of optical axis in world

    // ----------------------------------------------------------------
    void rebuildRotation()
    {
        // R_fix: OpenCV cam → ENU body at zero attitude
        Mat3 R_fix;
        R_fix.m[0][0]= 1; R_fix.m[0][2]= 0;   // East  = +X_cam
        R_fix.m[1][2]= 1;                        // North = +Z_cam
        R_fix.m[2][1]=-1;                        // Up    = -Y_cam

        Mat3 R_att = rotZ(-cam_.yaw) * rotX(-cam_.pitch) * rotY(cam_.roll);
        R_wc_ = R_att * R_fix;
    }

    // ----------------------------------------------------------------
    // Undistort one pixel → normalised undistorted coords (xn, yn).
    // Iterative fixed-point solve of the OpenCV distortion model.
    // Converges in ~5 iterations for typical lenses (tol 1e-9 in ≤20).
    // No heap allocation.
    // ----------------------------------------------------------------
    void undistort(double u_d, double v_d, double& xn, double& yn) const
    {
        // Normalised distorted coords = initial guess
        xn = (u_d - cam_.cx) / cam_.fx;
        yn = (v_d - cam_.cy) / cam_.fy;
        const double xd_t = xn, yd_t = yn;

        for (int i = 0; i < 20; i++) {
            double r2 = xn*xn + yn*yn;
            double r4 = r2*r2, r6 = r4*r2;
            double rad = 1.0 + dist_.k1*r2 + dist_.k2*r4 + dist_.k3*r6;
            double xd  = xn*rad + 2.0*dist_.p1*xn*yn       + dist_.p2*(r2+2.0*xn*xn);
            double yd  = yn*rad + dist_.p1*(r2+2.0*yn*yn)   + 2.0*dist_.p2*xn*yn;
            double ex  = xd_t - xd, ey = yd_t - yd;
            xn += ex; yn += ey;
            if (ex*ex + ey*ey < 1e-18) break;  // 1e-9 squared
        }
    }

    // ----------------------------------------------------------------
    IntersectionResult projectOne(const Pixel& px, const Vec3& origin) const
    {
        IntersectionResult res;
        res.u = px.u;
        res.v = px.v;

        // 1. Undistort
        double xn, yn;
        undistort(px.u, px.v, xn, yn);

        // 2. Ray in world frame
        // Normalise the ray immediately so that t IS the metric distance.
        Vec3 ray = (R_wc_ * Vec3{xn, yn, 1.0}).normalized();

        // 3. Intersect with Z = 0
        // t = -origin.z / ray.z   (ray is unit, so t = distance directly)
        constexpr double EPS = 1e-9;
        if (ray.z >= -EPS) {
            res.valid  = false;
            res.reason = (ray.z > EPS) ? "Ray points upward (pixel above horizon)"
                                       : "Ray parallel to ground";
            return res;
        }

        double t   = -origin.z / ray.z;   // metric distance (ray is unit)
        Vec3   hit = origin + ray * t;

        res.valid    = true;
        res.world_x  = hit.x;
        res.world_y  = hit.y;
        res.distance = t;                  // no sqrt needed — ray is already unit
        return res;
    }
};