#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <cmath>

namespace izmir {

struct Vec2d {
    double x = 0.0; // Dogu (+X) / Bati (-X) metre
    double z = 0.0; // Guney (+Z) / Kuzey (-Z) metre (OpenGL sag-el ekseni)
};

struct Vec3f {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    Vec3f() = default;
    Vec3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    Vec3f operator+(const Vec3f& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3f operator-(const Vec3f& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3f operator*(float s) const { return {x * s, y * s, z * s}; }
};

struct GeoCoord {
    double lat = 0.0; // Derece (Kuzey +)
    double lon = 0.0; // Derece (Dogu +)
};

struct GeoBounds {
    double minLat = 0.0, minLon = 0.0;
    double maxLat = 0.0, maxLon = 0.0;
    bool contains(double lat, double lon) const {
        return lat >= minLat && lat <= maxLat && lon >= minLon && lon <= maxLon;
    }
};

} // namespace izmir
