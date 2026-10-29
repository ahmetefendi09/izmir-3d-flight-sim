#pragma once
#include "Types.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace izmir {

static constexpr uint32_t IZMB_MAGIC   = 0x424D5A49; // "IZMB"
static constexpr uint32_t IZMB_VERSION = 2;

enum class WayCategory : uint8_t {
    Motorway     = 0,
    Primary      = 1,
    Secondary    = 2,
    Residential  = 3,
    IzbanRailway = 4,
    Runway       = 5
};

enum class BuildingCategory : uint8_t {
    StandardApartment    = 0,
    HighRiseEstate       = 1,
    GlassSkyscraper      = 2,
    CommercialIndustrial = 3,
    Mosque               = 4
};

#pragma pack(push, 1)
struct IzmbHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t demNX;
    uint32_t demNZ;
    float    minX, maxX;
    float    minZ, maxZ;
    uint32_t numBuildings;
    uint32_t numWays;
    uint32_t numStations;
};

struct StationRecord {
    char     code[8];
    char     name[32];
    float    x, y, z;
    float    yawRad;
    uint8_t  isUnderground;
};
#pragma pack(pop)

struct BuildingRecord {
    BuildingCategory      category = BuildingCategory::StandardApartment;
    uint8_t               roofType = 1;
    float                 baseElevation = 0.0f;
    float                 height = 15.0f;
    uint32_t              colorSeed = 0;
    std::vector<Vec3f>    footprintXZ;
    std::vector<uint16_t> roofIndices;
};

struct WayRibbonRecord {
    WayCategory        category = WayCategory::Residential;
    float              widthMeters = 7.0f;
    bool               isBridge = false;
    std::vector<Vec3f> centerline;
};

class IzmbWorldData {
public:
    IzmbHeader                  header;
    std::vector<float>          demHeights;
    std::vector<uint8_t>        demWaterMask;
    std::vector<BuildingRecord> buildings;
    std::vector<WayRibbonRecord>ways;
    std::vector<StationRecord>  stations;

    IzmbWorldData() {
        std::memset(&header, 0, sizeof(header));
        header.magic   = IZMB_MAGIC;
        header.version = IZMB_VERSION;
        header.minX    = -15000.0f;
        header.maxX    =  12000.0f;
        header.minZ    = -23000.0f;
        header.maxZ    =  18000.0f;
    }

    void allocateDemGrid(uint32_t nx, uint32_t nz) {
        header.demNX = nx;
        header.demNZ = nz;
        demHeights.assign(nx * nz, 0.0f);
        demWaterMask.assign(nx * nz, 1);
    }

    // GPU GL_TRIANGLES (i00-i01-i10 ve i10-i01-i11) EBO yuzeyiyle %100 ayni duzlem denklemi!
    // Yollar ve raylar yokuslarda asla topragin altina girmez veya havada kalmaz!
    float sampleHeightAt(float wx, float wz, bool& outIsWater) const {
        if (header.demNX < 2 || header.demNZ < 2 || demHeights.empty()) {
            outIsWater = true;
            return 0.0f;
        }
        float u = (wx - header.minX) / (header.maxX - header.minX);
        float v = (wz - header.minZ) / (header.maxZ - header.minZ);
        if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) {
            outIsWater = true;
            return 0.0f;
        }

        float fx = u * (header.demNX - 1);
        float fz = v * (header.demNZ - 1);
        int gx = std::clamp((int)fx, 0, (int)header.demNX - 2);
        int gz = std::clamp((int)fz, 0, (int)header.demNZ - 2);
        float tx = fx - gx;
        float tz = fz - gz;

        uint32_t i00 = gz * header.demNX + gx;
        uint32_t i10 = i00 + 1;
        uint32_t i01 = (gz + 1) * header.demNX + gx;
        uint32_t i11 = i01 + 1;

        float wSum = (demWaterMask[i00] ? 1.0f : 0.0f) * (1.0f - tx) * (1.0f - tz) +
                     (demWaterMask[i10] ? 1.0f : 0.0f) * tx * (1.0f - tz) +
                     (demWaterMask[i01] ? 1.0f : 0.0f) * (1.0f - tx) * tz +
                     (demWaterMask[i11] ? 1.0f : 0.0f) * tx * tz;
        outIsWater = (wSum > 0.5f);

        float h00 = demHeights[i00];
        float h10 = demHeights[i10];
        float h01 = demHeights[i01];
        float h11 = demHeights[i11];

        // TerrainRenderer EBO ucgen kesimiyle birebir barycentric yukseklik:
        if (tx + tz <= 1.0f) {
            return h00 + tx * (h10 - h00) + tz * (h01 - h00);
        } else {
            return h11 + (1.0f - tx) * (h01 - h11) + (1.0f - tz) * (h10 - h11);
        }
    }

    bool saveToBinary(const std::string& path) {
        header.numBuildings = (uint32_t)buildings.size();
        header.numWays      = (uint32_t)ways.size();
        header.numStations  = (uint32_t)stations.size();

        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out.good()) return false;

        out.write(reinterpret_cast<const char*>(&header), sizeof(IzmbHeader));
        out.write(reinterpret_cast<const char*>(demHeights.data()), demHeights.size() * sizeof(float));
        out.write(reinterpret_cast<const char*>(demWaterMask.data()), demWaterMask.size() * sizeof(uint8_t));

        for (const auto& b : buildings) {
            uint8_t cat = static_cast<uint8_t>(b.category);
            uint16_t nPts = (uint16_t)b.footprintXZ.size();
            uint16_t nIdx = (uint16_t)b.roofIndices.size();
            out.write(reinterpret_cast<const char*>(&cat), sizeof(uint8_t));
            out.write(reinterpret_cast<const char*>(&b.roofType), sizeof(uint8_t));
            out.write(reinterpret_cast<const char*>(&b.baseElevation), sizeof(float));
            out.write(reinterpret_cast<const char*>(&b.height), sizeof(float));
            out.write(reinterpret_cast<const char*>(&b.colorSeed), sizeof(uint32_t));
            out.write(reinterpret_cast<const char*>(&nPts), sizeof(uint16_t));
            out.write(reinterpret_cast<const char*>(&nIdx), sizeof(uint16_t));
            out.write(reinterpret_cast<const char*>(b.footprintXZ.data()), nPts * sizeof(Vec3f));
            out.write(reinterpret_cast<const char*>(b.roofIndices.data()), nIdx * sizeof(uint16_t));
        }

        for (const auto& w : ways) {
            uint8_t cat = static_cast<uint8_t>(w.category);
            uint8_t brg = w.isBridge ? 1 : 0;
            uint16_t nPts = (uint16_t)w.centerline.size();
            out.write(reinterpret_cast<const char*>(&cat), sizeof(uint8_t));
            out.write(reinterpret_cast<const char*>(&brg), sizeof(uint8_t));
            out.write(reinterpret_cast<const char*>(&w.widthMeters), sizeof(float));
            out.write(reinterpret_cast<const char*>(&nPts), sizeof(uint16_t));
            out.write(reinterpret_cast<const char*>(w.centerline.data()), nPts * sizeof(Vec3f));
        }

        if (!stations.empty()) {
            out.write(reinterpret_cast<const char*>(stations.data()), stations.size() * sizeof(StationRecord));
        }
        return out.good();
    }

    bool loadFromBinary(const std::string& path) {
        std::ifstream in(path, std::ios::binary);
        if (!in.good()) return false;

        in.read(reinterpret_cast<char*>(&header), sizeof(IzmbHeader));
        if (header.magic != IZMB_MAGIC || header.version != IZMB_VERSION) return false;

        size_t gridCount = (size_t)header.demNX * (size_t)header.demNZ;
        demHeights.resize(gridCount);
        demWaterMask.resize(gridCount);
        in.read(reinterpret_cast<char*>(demHeights.data()), gridCount * sizeof(float));
        in.read(reinterpret_cast<char*>(demWaterMask.data()), gridCount * sizeof(uint8_t));

        buildings.resize(header.numBuildings);
        for (uint32_t i = 0; i < header.numBuildings; ++i) {
            auto& b = buildings[i];
            uint8_t cat = 0;
            uint16_t nPts = 0, nIdx = 0;
            in.read(reinterpret_cast<char*>(&cat), sizeof(uint8_t));
            in.read(reinterpret_cast<char*>(&b.roofType), sizeof(uint8_t));
            in.read(reinterpret_cast<char*>(&b.baseElevation), sizeof(float));
            in.read(reinterpret_cast<char*>(&b.height), sizeof(float));
            in.read(reinterpret_cast<char*>(&b.colorSeed), sizeof(uint32_t));
            in.read(reinterpret_cast<char*>(&nPts), sizeof(uint16_t));
            in.read(reinterpret_cast<char*>(&nIdx), sizeof(uint16_t));
            b.category = static_cast<BuildingCategory>(cat);
            b.footprintXZ.resize(nPts);
            b.roofIndices.resize(nIdx);
            in.read(reinterpret_cast<char*>(b.footprintXZ.data()), nPts * sizeof(Vec3f));
            in.read(reinterpret_cast<char*>(b.roofIndices.data()), nIdx * sizeof(uint16_t));
        }

        ways.resize(header.numWays);
        for (uint32_t i = 0; i < header.numWays; ++i) {
            auto& w = ways[i];
            uint8_t cat = 0, brg = 0;
            uint16_t nPts = 0;
            in.read(reinterpret_cast<char*>(&cat), sizeof(uint8_t));
            in.read(reinterpret_cast<char*>(&brg), sizeof(uint8_t));
            in.read(reinterpret_cast<char*>(&w.widthMeters), sizeof(float));
            in.read(reinterpret_cast<char*>(&nPts), sizeof(uint16_t));
            w.category = static_cast<WayCategory>(cat);
            w.isBridge = (brg != 0);
            w.centerline.resize(nPts);
            in.read(reinterpret_cast<char*>(w.centerline.data()), nPts * sizeof(Vec3f));
        }

        stations.resize(header.numStations);
        if (header.numStations > 0) {
            in.read(reinterpret_cast<char*>(stations.data()), header.numStations * sizeof(StationRecord));
        }
        return in.good();
    }
};

} // namespace izmir
