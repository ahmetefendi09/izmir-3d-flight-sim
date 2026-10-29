#pragma once
#include "Types.hpp"
#include <cmath>
#include <algorithm>

namespace izmir {

class GeoProjection {
public:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEG2RAD = PI / 180.0;
    static constexpr double RAD2DEG = 180.0 / PI;

    static constexpr double WGS84_A  = 6378137.0;
    static constexpr double WGS84_F  = 1.0 / 298.257223563;
    static constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F;

    // Konak Saat Kulesi Kesin Referans Noktasi
    static constexpr double ORIGIN_LAT = 38.418898;
    static constexpr double ORIGIN_LON = 27.128677;

    static constexpr GeoBounds IZMIR_MASTER_BOUNDS = {
        38.2600, 26.9600,
        38.6250, 27.2650
    };

private:
    double m_metersPerDegLat = 111045.0;
    double m_metersPerDegLon = 87430.0;

public:
    GeoProjection() {
        double latRad = ORIGIN_LAT * DEG2RAD;
        double sinLat = std::sin(latRad);
        double cosLat = std::cos(latRad);
        double denom  = std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);

        double rMeridian = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom);
        double rPrimeVertical = WGS84_A / denom;

        m_metersPerDegLat = rMeridian * DEG2RAD;
        m_metersPerDegLon = rPrimeVertical * cosLat * DEG2RAD;
    }

    Vec2d latLonToWorldXZ(double lat, double lon) const {
        double dLat = lat - ORIGIN_LAT;
        double dLon = lon - ORIGIN_LON;
        return Vec2d{
             dLon * m_metersPerDegLon, // +X = Dogu
            -dLat * m_metersPerDegLat  // -Z = Kuzey
        };
    }

    GeoCoord worldXZToLatLon(double xMeters, double zMeters) const {
        double lat = ORIGIN_LAT - (zMeters / m_metersPerDegLat);
        double lon = ORIGIN_LON + (xMeters / m_metersPerDegLon);
        return GeoCoord{lat, lon};
    }

    double distanceMeters(GeoCoord a, GeoCoord b) const {
        Vec2d pa = latLonToWorldXZ(a.lat, a.lon);
        Vec2d pb = latLonToWorldXZ(b.lat, b.lon);
        return std::hypot(pb.x - pa.x, pb.z - pa.z);
    }
};

} // namespace izmir
