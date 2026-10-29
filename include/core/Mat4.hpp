#pragma once
#include "Types.hpp"
#include <cmath>
#include <cstring>

namespace izmir {

struct Mat4 {
    float m[16];

    static Mat4 identity() {
        Mat4 r;
        std::memset(r.m, 0, sizeof(r.m));
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }
};

inline float dot3(const Vec3f& a, const Vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3f cross3(const Vec3f& a, const Vec3f& b) {
    return Vec3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline Vec3f normalize3(const Vec3f& v) {
    float len = std::sqrt(dot3(v, v));
    return (len > 1e-6f) ? (v * (1.0f / len)) : Vec3f(0.0f, 1.0f, 0.0f);
}

} // namespace izmir
