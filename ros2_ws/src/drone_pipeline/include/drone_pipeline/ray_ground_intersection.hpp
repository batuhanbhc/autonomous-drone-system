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
 *  yaw   – ENU yaw, counter-clockwise from East.
 *             0=East, 90=North, 180=West, -90=South
 *
 *  pitch – nose-down positive.
 *             0=horizontal, +90=straight down, -90=straight up
 *
 *  roll  – right wing down positive.
 *             0=level, +30=right side down
 *
 *  mount_pitch – fixed camera mount pitch relative to body, nose-down positive.
 *
 * ============================================================
 * ROTATION
 * ============================================================
 *
 *  R_fix  : OpenCV cam axes → ENU world at zero attitude (cam faces East)
 *             East  (+X_enu) = +Z_cam
 *             North (+Y_enu) = -X_cam
 *             Up    (+Z_enu) = -Y_cam
 *
 *  R_body : Rz(yaw) * Ry(pitch) * Rx(roll)
 *  R_mount: Ry(mount_pitch)
 *
 *  R_wc   : R_body * R_mount * R_fix   (precomputed once per pose update)
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
    double yaw   = 0.0;  ///< ENU yaw, CCW from East (rad)
    double pitch = 0.0;  ///< nose-down positive (rad)
    double roll  = 0.0;  ///< right-wing-down positive (rad)
    double mount_pitch = 0.0;  ///< fixed camera tilt relative to body (rad)
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
                 double yaw, double pitch, double roll, double mount_pitch)
    {
        cam_.x = x; cam_.y = y; cam_.z = z;
        cam_.yaw = yaw; cam_.pitch = pitch; cam_.roll = roll;
        cam_.mount_pitch = mount_pitch;
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

    // ----------------------------------------------------------------
    void rebuildRotation()
    {
        // R_fix: OpenCV cam → ENU world when yaw=pitch=roll=0 (camera faces East)
        Mat3 R_fix;
        R_fix.m[0][2]= 1;   // East  = +Z_cam
        R_fix.m[1][0]=-1;   // North = -X_cam
        R_fix.m[2][1]=-1;   // Up    = -Y_cam

        Mat3 R_body = rotZ(cam_.yaw) * rotY(cam_.pitch) * rotX(cam_.roll);
        Mat3 R_mount = rotY(cam_.mount_pitch);
        R_wc_ = R_body * R_mount * R_fix;
    }

    // ----------------------------------------------------------------
    // Undistort one pixel → normalised undistorted coords (xn, yn).
    // Iterative fixed-point solve of the OpenCV distortion model.
    // ----------------------------------------------------------------
    void undistort(double u_d, double v_d, double& xn, double& yn) const
    {
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
            if (ex*ex + ey*ey < 1e-18) break;
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
        Vec3 ray = (R_wc_ * Vec3{xn, yn, 1.0}).normalized();

        // 3. Intersect with Z = 0
        constexpr double EPS = 1e-9;
        if (ray.z >= -EPS) {
            res.valid  = false;
            res.reason = (ray.z > EPS) ? "Ray points upward (pixel above horizon)"
                                       : "Ray parallel to ground";
            return res;
        }

        double t   = -origin.z / ray.z;
        Vec3   hit = origin + ray * t;

        res.valid    = true;
        res.world_x  = hit.x;
        res.world_y  = hit.y;
        res.distance = t;
        return res;
    }
};
