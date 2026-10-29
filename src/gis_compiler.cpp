#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <curl/curl.h>
#include <png.h>

#include "core/GeoProjection.hpp"
#include "core/IzmbFormat.hpp"
#include "mapbox/earcut.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace izmir;

struct RawDemTile {
    int w = 0;
    int h = 0;
    std::vector<uint8_t> rgb;
};

static size_t curlWriteCb(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t total = size * nmemb;
    auto* buf = static_cast<std::vector<uint8_t>*>(userp);
    const uint8_t* ptr = static_cast<const uint8_t*>(contents);
    buf->insert(buf->end(), ptr, ptr + total);
    return total;
}

bool downloadBinaryOrCache(CURL* curl, const std::string& url, const std::string& cacheFile,
                           std::vector<uint8_t>& outData, long timeoutSec = 20L) {
    outData.clear();
    {
        std::ifstream in(cacheFile, std::ios::binary | std::ios::ate);
        if (in.good()) {
            std::streamsize sz = in.tellg();
            if (sz > 128) {
                in.seekg(0, std::ios::beg);
                outData.resize((size_t)sz);
                if (in.read(reinterpret_cast<char*>(outData.data()), sz)) return true;
            }
        }
    }
    if (!curl) return false;
    outData.clear();
    curl_easy_reset(curl);
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &outData);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "IzmirOvertureCompiler/13.0");
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeoutSec);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    CURLcode res = curl_easy_perform(curl);
    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
    if (res == CURLE_OK && httpCode == 200 && outData.size() > 128) {
        std::ofstream out(cacheFile, std::ios::binary);
        if (out.good()) out.write(reinterpret_cast<const char*>(outData.data()), outData.size());
        return true;
    }
    return false;
}

bool decodePngRGB(const std::vector<uint8_t>& pngBytes, int& outW, int& outH, std::vector<uint8_t>& outRGB) {
    if (pngBytes.size() < 24 || pngBytes[0] != 0x89 || pngBytes[1] != 'P') return false;
    png_image img;
    std::memset(&img, 0, sizeof(img));
    img.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_memory(&img, pngBytes.data(), pngBytes.size())) return false;
    img.format = PNG_FORMAT_RGB;
    outW = (int)img.width;
    outH = (int)img.height;
    outRGB.resize(PNG_IMAGE_SIZE(img));
    if (!png_image_finish_read(&img, nullptr, outRGB.data(), 0, nullptr)) {
        png_image_free(&img);
        return false;
    }
    return true;
}

void latLonToZoom12TilePixel(double latDeg, double lonDeg, int& outTx, int& outTy, float& outPx, float& outPy) {
    double n = 4096.0;
    double latRad = latDeg * GeoProjection::DEG2RAD;
    double fx = (lonDeg + 180.0) / 360.0 * n;
    double fy = (1.0 - std::log(std::tan(latRad) + 1.0 / std::cos(latRad)) / GeoProjection::PI) * 0.5 * n;
    outTx = (int)std::floor(fx);
    outTy = (int)std::floor(fy);
    outPx = (float)((fx - outTx) * 255.0);
    outPy = (float)((fy - outTy) * 255.0);
}

std::string sanitizeTurkishUpper(const std::string& s) {
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        unsigned char c = (unsigned char)s[i];
        if (c == 0xC3 && i + 1 < s.size()) {
            unsigned char c2 = (unsigned char)s[++i];
            if (c2 == 0x87 || c2 == 0xA7) out += 'C';
            else if (c2 == 0x96 || c2 == 0xB6) out += 'O';
            else if (c2 == 0x9C || c2 == 0xBC) out += 'U';
        } else if (c == 0xC4 && i + 1 < s.size()) {
            unsigned char c2 = (unsigned char)s[++i];
            if (c2 == 0x9E || c2 == 0x9F) out += 'G';
            else if (c2 == 0xB0 || c2 == 0xB1) out += 'I';
        } else if (c == 0xC5 && i + 1 < s.size()) {
            unsigned char c2 = (unsigned char)s[++i];
            if (c2 == 0x9E || c2 == 0x9F) out += 'S';
        } else if (c >= 32 && c < 127) {
            out += (char)std::toupper(c);
        }
    }
    return out;
}

void snapStationToNearestRail(StationRecord& st, const IzmbWorldData& world) {
    float bestDistSq = 1e18f;
    float bestX = st.x, bestZ = st.z, bestYaw = st.yawRad;
    bool found = false;

    for (const auto& w : world.ways) {
        if (w.category != WayCategory::IzbanRailway) continue;
        for (size_t i = 0; i + 1 < w.centerline.size(); ++i) {
            const Vec3f& a = w.centerline[i];
            const Vec3f& b = w.centerline[i + 1];
            float dx = b.x - a.x, dz = b.z - a.z;
            float lenSq = dx * dx + dz * dz;
            if (lenSq < 1.0f) continue;

            float t = std::clamp(((st.x - a.x) * dx + (st.z - a.z) * dz) / lenSq, 0.0f, 1.0f);
            float projX = a.x + t * dx, projZ = a.z + t * dz;
            float dSq = (st.x - projX) * (st.x - projX) + (st.z - projZ) * (st.z - projZ);
            if (dSq < bestDistSq) {
                bestDistSq = dSq;
                bestX = projX; bestZ = projZ;
                bestYaw = std::atan2(dx, dz);
                found = true;
            }
        }
    }
    if (found && bestDistSq < 350.0f * 350.0f) {
        st.x = bestX; st.z = bestZ;
        bool wat = false;
        st.y = world.sampleHeightAt(st.x, st.z, wat);
        st.yawRad = bestYaw;
    }
}

// Tum tasit yollarini, site ici yollari (service), carsi sokaklarini (pedestrian) ve
// siniflandirilmamis (unknown/unclassified) ara sokaklari kabul et! (Hicbir sokak yarida kesilmez!)
bool isValidRoadClass(const std::string& c, const std::string& sub) {
    if (c == "footway" || c == "steps" || c == "path" || c == "cycleway" || c == "bridleway") {
        return false;
    }
    if (c == "service" && (sub == "driveway" || sub == "parking_aisle")) {
        return false;
    }
    return (c == "motorway" || c == "motorway_link" ||
            c == "trunk"    || c == "trunk_link"    ||
            c == "primary"  || c == "primary_link"  ||
            c == "secondary"|| c == "secondary_link"||
            c == "tertiary" || c == "tertiary_link" ||
            c == "residential" || c == "unclassified" ||
            c == "living_street" || c == "service" ||
            c == "pedestrian" || c == "unknown");
}

// ============================================================================
// IZMIR IMAR PLANI VE YAPI ADASI UYUMLU BINA YUKSEKLIK / KAT MODELI
// * Ayni yapi adasindaki (100m x 100m blok) bitisik nizam apartmanlar uyumlu
//   kat yuksekligine sahip olur (biri 2 katli, yanindaki 7 katli gibi sapitmaz!)
// ============================================================================
void assignRealisticBuildingMetrics(BuildingRecord& bRec, float areaM2, float maxSpanM,
                                    float avgX, float avgZ,
                                    const std::string& bClass, int levels, float explicitH) {
    // 120m x 120m Yapi Adasi Tohumu (Ayni sokaktaki binalarin uyumlu yukseklikte olmasi icin)
    int blockX = (int)std::floor(avgX / 120.0f);
    int blockZ = (int)std::floor(avgZ / 120.0f);
    uint32_t blockHash = (uint32_t)(blockX * 73856093 ^ blockZ * 19349663);

    bool inBayrakliCBD = (avgX > 1800.0f && avgX < 5800.0f && avgZ > -5200.0f && avgZ < -1200.0f);
    bool inHighRiseEstateZone =
        (avgZ < -5800.0f && avgX > -7200.0f && avgX < -1800.0f) || // Mavisehir / Bostanli / Cigli / Egekent 2 / Evka-5
        (avgX > 4500.0f && avgZ > -6500.0f && avgZ < -2500.0f);    // Bornova / Ozkanlar / Evka-4
    bool inUrbanCore =
        (avgZ > -6500.0f && avgZ < 4500.0f && avgX > -6500.0f && avgX < 8500.0f); // Karsiyaka, Alsancak, Konak, Goztepe, Hatay, Buca, Bornova

    if (bClass == "mosque" || bClass == "religious") {
        bRec.category = BuildingCategory::Mosque;
        bRec.roofType = 2;
        bRec.height   = (explicitH > 4.0f) ? explicitH : 16.0f;
        return;
    }

    if (bClass == "industrial" || bClass == "warehouse" || bClass == "retail" ||
        bClass == "commercial" || bClass == "school" || bClass == "hospital" ||
        bClass == "university" || areaM2 > 1200.0f || maxSpanM > 48.0f) {
        if (inBayrakliCBD && (explicitH > 38.0f || levels >= 12 || (areaM2 > 320.0f && areaM2 < 2400.0f && (blockHash % 3 == 0)))) {
            bRec.category = BuildingCategory::GlassSkyscraper;
            bRec.roofType = 0;
            bRec.height   = (explicitH > 4.0f) ? explicitH : (levels > 0 ? levels * 3.6f : (58.0f + (bRec.colorSeed % 75)));
            return;
        }
        bRec.category = BuildingCategory::CommercialIndustrial;
        bRec.roofType = 0;
        bRec.height   = (explicitH > 3.5f) ? std::min(explicitH, 24.0f) : (levels > 0 ? levels * 3.6f : 10.5f);
        return;
    }

    if (explicitH > 42.0f || levels >= 14) {
        bRec.category = BuildingCategory::GlassSkyscraper;
        bRec.roofType = 0;
        bRec.height   = (explicitH > 4.0f) ? explicitH : (levels * 3.5f);
        return;
    }

    if (explicitH > 3.2f) {
        bRec.height   = explicitH;
        bRec.category = (explicitH > 22.0f) ? BuildingCategory::HighRiseEstate : BuildingCategory::StandardApartment;
        bRec.roofType = (explicitH > 24.0f) ? 0 : 1;
        return;
    }
    if (levels > 0) {
        bRec.height   = levels * 3.1f;
        bRec.category = (levels >= 8) ? BuildingCategory::HighRiseEstate : BuildingCategory::StandardApartment;
        bRec.roofType = (levels >= 8) ? 0 : 1;
        return;
    }

    // Yapi Adasi Uyumlu Kat Tahmini:
    if (areaM2 < 34.0f) {
        // Kucuk musfetti/garaj/kulube (1 kat)
        bRec.category = BuildingCategory::StandardApartment;
        bRec.roofType = 1;
        bRec.height   = 3.4f;
    } else if (inHighRiseEstateZone && areaM2 >= 145.0f) {
        // Mavisehir / Egekent 2 / Ulukent / Bostanli / Evka Site Bloklari (8 - 12 kat, ayni adada uyumlu!)
        int baseBlockFloors = 8 + (int)(blockHash % 4);
        int delta = (int)(bRec.colorSeed % 2);
        bRec.category = BuildingCategory::HighRiseEstate;
        bRec.roofType = 0;
        bRec.height   = (baseBlockFloors + delta) * 3.1f;
    } else if (inUrbanCore && areaM2 >= 58.0f) {
        // Merkez Izmir (Karsiyaka, Alsancak, Konak, Goztepe, Buca, Bornova) Bitisik Nizam Apartmanlar (5 - 7 kat)
        int baseBlockFloors = 5 + (int)(blockHash % 3);
        bRec.category = BuildingCategory::StandardApartment;
        bRec.roofType = 1;
        bRec.height   = baseBlockFloors * 3.1f;
    } else {
        // Dis Ilceler / Yamac Mahalleleri (Menemen, Ulukent, Koyundere, Kadifekale, Karabaglar, Gaziemir)
        int baseBlockFloors = (areaM2 >= 110.0f) ? (4 + (int)(blockHash % 3)) : (2 + (int)(blockHash % 2));
        bRec.category = BuildingCategory::StandardApartment;
        bRec.roofType = 1;
        bRec.height   = baseBlockFloors * 3.1f;
    }
}

int main() {
    std::cout << "============================================================================\n";
    std::cout << "  IZMIR 3D SIMULATOR - OVERTURE KESINTISIZ HARITA DERLEYICISI v13.0\n";
    std::cout << "============================================================================\n\n";

    curl_global_init(CURL_GLOBAL_ALL);
    CURL* curl = curl_easy_init();
    GeoProjection proj;
    IzmbWorldData world;
    world.allocateDemGrid(512, 768);

    const int DEM_X0 = 2354, DEM_X1 = 2358, DEM_Y0 = 1570, DEM_Y1 = 1576;
    const int DEM_W = DEM_X1 - DEM_X0 + 1, DEM_H = DEM_Y1 - DEM_Y0 + 1;
    std::vector<RawDemTile> demTiles(DEM_W * DEM_H);

    for (int ty = DEM_Y0; ty <= DEM_Y1; ++ty) {
        for (int tx = DEM_X0; tx <= DEM_X1; ++tx) {
            char url[256], cache[256];
            std::snprintf(url, sizeof(url), "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/12/%d/%d.png", tx, ty);
            std::snprintf(cache, sizeof(cache), "../data/raw_osm/srtm_z12_%d_%d.png", tx, ty);
            std::vector<uint8_t> buf;
            RawDemTile& dt = demTiles[(ty - DEM_Y0) * DEM_W + (tx - DEM_X0)];
            if (downloadBinaryOrCache(curl, url, cache, buf, 15L)) decodePngRGB(buf, dt.w, dt.h, dt.rgb);
        }
    }

    auto readSrtmAtLatLon = [&](double lat, double lon) -> float {
        int tx = 0, ty = 0; float px = 0.0f, py = 0.0f;
        latLonToZoom12TilePixel(lat, lon, tx, ty, px, py);
        if (tx < DEM_X0 || tx > DEM_X1 || ty < DEM_Y0 || ty > DEM_Y1) return 0.0f;
        const RawDemTile& dt = demTiles[(ty - DEM_Y0) * DEM_W + (tx - DEM_X0)];
        if (dt.w != 256 || dt.h != 256) return 2.0f;
        int x0 = std::clamp((int)px, 0, 254), y0 = std::clamp((int)py, 0, 254);
        float wx = px - x0, wy = py - y0;
        auto elevPx = [&](int x, int y) -> float {
            int idx = (y * 256 + x) * 3;
            return (dt.rgb[idx] * 256.0f + dt.rgb[idx + 1] + dt.rgb[idx + 2] / 256.0f) - 32768.0f;
        };
        float top = elevPx(x0, y0) * (1.0f - wx) + elevPx(x0 + 1, y0) * wx;
        float bot = elevPx(x0, y0 + 1) * (1.0f - wx) + elevPx(x0 + 1, y0 + 1) * wx;
        return top * (1.0f - wy) + bot * wy;
    };

    uint32_t NX = world.header.demNX, NZ = world.header.demNZ;
    float minX = world.header.minX, spanX = world.header.maxX - world.header.minX;
    float minZ = world.header.minZ, spanZ = world.header.maxZ - world.header.minZ;

    for (uint32_t gz = 0; gz < NZ; ++gz) {
        float v = (float)gz / (NZ - 1);
        float wz = world.header.minZ + v * spanZ;
        for (uint32_t gx = 0; gx < NX; ++gx) {
            float u = (float)gx / (NX - 1);
            float wx = world.header.minX + u * spanX;
            GeoCoord ll = proj.worldXZToLatLon(wx, wz);
            float rawElev = readSrtmAtLatLon(ll.lat, ll.lon);
            uint32_t idx = gz * NX + gx;
            if (rawElev <= 0.4f) { world.demWaterMask[idx] = 1; world.demHeights[idx] = 0.0f; }
            else { world.demWaterMask[idx] = 0; world.demHeights[idx] = std::max(2.6f, rawElev); }
        }
    }

    auto ensureDryCellAt = [&](float wx, float wz) {
        int gx = std::clamp((int)(((wx - minX) / spanX) * (NX - 1)), 0, (int)NX - 1);
        int gz = std::clamp((int)(((wz - minZ) / spanZ) * (NZ - 1)), 0, (int)NZ - 1);
        uint32_t idx = (uint32_t)gz * NX + (uint32_t)gx;
        if (world.demWaterMask[idx] != 0 || world.demHeights[idx] < 2.6f) {
            world.demWaterMask[idx] = 0;
            world.demHeights[idx]   = std::max(world.demHeights[idx], 2.8f);
        }
    };

    // ========================================================================
    // ADIM 1: OVERTURE MAPS ULASIM AGI (SIFIR SON DUGUM KAYBI + ALT GECIT KORUMA!)
    // ========================================================================
    uint32_t overtureWays = 0;
    {
        std::ifstream ovRoadFile("../data/overture/izmir_roads.geojsonseq");
        if (ovRoadFile.good()) {
            std::cout << "[ADIM 1/3] Overture Maps Tum Yol, Kavsak, Alt Gecit ve Ray Agi Yukleniyor...\n";
            std::string line;
            while (std::getline(ovRoadFile, line)) {
                if (line.size() < 20) continue;
                size_t startPos = line.find('{');
                if (startPos == std::string::npos) continue;

                json feat = json::parse(line.begin() + startPos, line.end(), nullptr, false);
                if (feat.is_discarded() || !feat.contains("geometry") || !feat["geometry"].is_object()) continue;
                const auto& geom = feat["geometry"];
                if (geom.value("type", "") != "LineString" || !geom.contains("coordinates")) continue;
                const auto& coords = geom["coordinates"];
                if (!coords.is_array() || coords.size() < 2) continue;

                std::string subtype = "", rClass = "", rSubclass = "";
                bool isBridge = false, isTunnel = false;
                if (feat.contains("properties") && feat["properties"].is_object()) {
                    const auto& props = feat["properties"];
                    if (props.contains("subtype") && props["subtype"].is_string())   subtype   = props["subtype"];
                    if (props.contains("class") && props["class"].is_string())       rClass    = props["class"];
                    if (props.contains("subclass") && props["subclass"].is_string()) rSubclass = props["subclass"];
                    std::string propDump = props.dump();
                    if (propDump.find("\"is_tunnel\"") != std::string::npos) isTunnel = true;
                    if (propDump.find("\"is_bridge\"") != std::string::npos) isBridge = true;
                }

                bool isRail = (subtype == "rail" && (rClass == "standard_gauge" || rClass == "rail" || rClass == "light_rail" || rClass == "tram" || rClass.empty()));
                bool isRoad = (subtype == "road" && isValidRoadClass(rClass, rSubclass));
                if (!isRail && !isRoad) continue;

                WayRibbonRecord wRec;
                wRec.isBridge = isBridge;

                if (isRail) {
                    wRec.category = WayCategory::IzbanRailway;
                    wRec.widthMeters = 5.0f;
                } else if (rClass == "motorway" || rClass == "trunk" || rClass == "motorway_link" || rClass == "trunk_link" ||
                           ((rClass == "motorway" || rClass == "trunk") && rSubclass == "link")) {
                    wRec.category = WayCategory::Motorway;
                    wRec.widthMeters = (rClass.find("_link") != std::string::npos || rSubclass == "link") ? 6.0f : 8.6f;
                } else if (rClass == "primary" || rClass == "primary_link") {
                    wRec.category = WayCategory::Primary;
                    wRec.widthMeters = (rClass.find("_link") != std::string::npos || rSubclass == "link") ? 5.6f : 7.2f;
                } else if (rClass == "secondary" || rClass == "secondary_link" || rClass == "tertiary" || rClass == "tertiary_link") {
                    wRec.category = WayCategory::Secondary;
                    wRec.widthMeters = 5.8f;
                } else {
                    wRec.category = WayCategory::Residential;
                    wRec.widthMeters = 4.4f;
                }

                float totalLengthM = 0.0f;
                size_t numPts = coords.size();
                for (size_t idx = 0; idx < numPts; ++idx) {
                    const auto& pt = coords[idx];
                    if (!pt.is_array() || pt.size() < 2) continue;
                    double lon = pt[0].get<double>();
                    double lat = pt[1].get<double>();
                    if (!GeoProjection::IZMIR_MASTER_BOUNDS.contains(lat, lon)) continue;
                    Vec2d xz = proj.latLonToWorldXZ(lat, lon);
                    float wx = (float)xz.x, wz = (float)xz.z;

                    if (!wRec.centerline.empty()) {
                        float d = std::hypot(wx - wRec.centerline.back().x, wz - wRec.centerline.back().z);
                        // SON DUGUMU (idx + 1 == numPts) ASLA ATLAMA! Kavsak baglantisi kopmaz!
                        if (d < 0.20f && (idx + 1 < numPts)) continue;
                        if (d > 900.0f) break;
                        totalLengthM += d;
                    }
                    ensureDryCellAt(wx, wz);
                    bool wat = false;
                    float gy = world.sampleHeightAt(wx, wz, wat);
                    wRec.centerline.push_back(Vec3f(wx, gy, wz));
                }

                // Sadece > 400m uzunlugundaki gercek dag tunellerini (Konak Tuneli, Bayrakli Otoyol Tunelleri) gizle!
                // 400m altindaki tum bulvar alt gecitlerini (Karsiyaka, Bayrakli, Halkapinar, Goztepe alt gecitleri) CIZ!
                if (isTunnel && !isRail && totalLengthM > 400.0f) {
                    continue;
                }

                if (wRec.centerline.size() >= 2) {
                    world.ways.push_back(std::move(wRec));
                    overtureWays++;
                }
            }
            std::cout << "  -> " << overtureWays << " Adet Kesintisiz Overture Yol/Kavsak/Ray Hatti Yuklendi.\n";
        }
    }

    // ========================================================================
    // ADIM 2: %100 SAF OVERTURE BINA POLIGONLARI (SIFIR BINA SILME!)
    // ========================================================================
    uint32_t overtureBldgs = 0;
    {
        std::ifstream ovBldgFile("../data/overture/izmir_buildings.geojsonseq");
        if (ovBldgFile.good()) {
            std::cout << "[ADIM 2/3] Overture Maps Tum Bitisik Nizam ve Site Binalari Eksiksiz Isleniyor...\n";
            std::string line;
            uint64_t seqId = 1;
            while (std::getline(ovBldgFile, line)) {
                if (line.size() < 20) continue;
                size_t startPos = line.find('{');
                if (startPos == std::string::npos) continue;

                json feat = json::parse(line.begin() + startPos, line.end(), nullptr, false);
                if (feat.is_discarded() || !feat.contains("geometry") || !feat["geometry"].is_object()) continue;

                const auto& geom = feat["geometry"];
                std::string gType = geom.value("type", "");
                if (!geom.contains("coordinates") || !geom["coordinates"].is_array()) continue;

                std::string bClass = "";
                int levels = 0;
                float explicitH = 0.0f;
                if (feat.contains("properties") && feat["properties"].is_object()) {
                    const auto& props = feat["properties"];
                    if (props.contains("class") && props["class"].is_string()) bClass = props["class"];
                    else if (props.contains("subtype") && props["subtype"].is_string()) bClass = props["subtype"];
                    if (props.contains("height") && props["height"].is_number()) explicitH = props["height"].get<float>();
                    if (props.contains("num_floors") && props["num_floors"].is_number()) levels = props["num_floors"].get<int>();
                }

                if (bClass == "bridge" || bClass == "greenhouse" || bClass == "ruins") {
                    continue;
                }

                auto processPolygonRing = [&](const json& outerRing) {
                    if (!outerRing.is_array() || outerRing.size() < 4) return;
                    BuildingRecord bRec;
                    bRec.colorSeed = (uint32_t)((seqId++) * 2654435761ULL);

                    using P2 = std::array<double, 2>;
                    std::vector<P2> ring2D;
                    float sumX = 0.0f, sumZ = 0.0f;
                    float bMinX = 1e9f, bMaxX = -1e9f, bMinZ = 1e9f, bMaxZ = -1e9f;

                    for (const auto& pt : outerRing) {
                        if (!pt.is_array() || pt.size() < 2) continue;
                        double lon = pt[0].get<double>();
                        double lat = pt[1].get<double>();
                        if (!GeoProjection::IZMIR_MASTER_BOUNDS.contains(lat, lon)) continue;
                        Vec2d xz = proj.latLonToWorldXZ(lat, lon);
                        float wx = (float)xz.x, wz = (float)xz.z;
                        if (!bRec.footprintXZ.empty() && std::hypot(wx - bRec.footprintXZ.back().x, wz - bRec.footprintXZ.back().z) < 0.20f) continue;
                        bRec.footprintXZ.push_back(Vec3f(wx, 0.0f, wz));
                        ring2D.push_back({xz.x, xz.z});
                        sumX += wx; sumZ += wz;
                        bMinX = std::min(bMinX, wx); bMaxX = std::max(bMaxX, wx);
                        bMinZ = std::min(bMinZ, wz); bMaxZ = std::max(bMaxZ, wz);
                    }

                    if (bRec.footprintXZ.size() > 3 && std::hypot(bRec.footprintXZ.front().x - bRec.footprintXZ.back().x, bRec.footprintXZ.front().z - bRec.footprintXZ.back().z) < 0.25f) {
                        bRec.footprintXZ.pop_back();
                        ring2D.pop_back();
                    }
                    int K = (int)bRec.footprintXZ.size();
                    if (K < 3 || K > 350) return;

                    float signedArea = 0.0f;
                    for (int i = 0; i < K; ++i) {
                        const Vec3f& p0 = bRec.footprintXZ[i];
                        const Vec3f& p1 = bRec.footprintXZ[(i + 1) % K];
                        signedArea += (p0.x * p1.z - p1.x * p0.z);
                    }
                    float areaM2 = 0.5f * std::fabs(signedArea);
                    float maxSpanM = std::max(bMaxX - bMinX, bMaxZ - bMinZ);
                    if (areaM2 < 14.0f || maxSpanM > 580.0f) return;

                    float avgX = sumX / K, avgZ = sumZ / K;
                    ensureDryCellAt(avgX, avgZ);
                    bool wat = false;
                    bRec.baseElevation = world.sampleHeightAt(avgX, avgZ, wat);

                    assignRealisticBuildingMetrics(bRec, areaM2, maxSpanM, avgX, avgZ, bClass, levels, explicitH);

                    std::vector<std::vector<P2>> wrap = { std::move(ring2D) };
                    bRec.roofIndices = mapbox::earcut<uint16_t>(wrap);
                    if (bRec.roofIndices.size() >= 3) {
                        world.buildings.push_back(std::move(bRec));
                        overtureBldgs++;
                    }
                };

                if (gType == "Polygon" && !geom["coordinates"].empty()) {
                    processPolygonRing(geom["coordinates"][0]);
                } else if (gType == "MultiPolygon") {
                    for (const auto& poly : geom["coordinates"]) {
                        if (poly.is_array() && !poly.empty()) processPolygonRing(poly[0]);
                    }
                }
            }
            std::cout << "  -> " << overtureBldgs << " Adet Gercek Overture Binasi Eksiksiz Eklendi!\n";
        }
    }

    // ========================================================================
    // ADIM 3: KIYI SERIDI VE 29 IZBAN ISTASYONU
    // ========================================================================
    std::cout << "[ADIM 3/3] Kiyi Seridi ve 29 IZBAN Istasyonu Kilitleniyor...\n";
    {
        std::ifstream csIn("../data/raw_osm/izmir_real_coast_and_stations.json", std::ios::binary);
        if (csIn.good()) {
            std::ostringstream ss; ss << csIn.rdbuf();
            json j = json::parse(ss.str(), nullptr, false);
            if (!j.is_discarded() && j.contains("elements") && j["elements"].is_array()) {
                for (const auto& el : j["elements"]) {
                    if (el.contains("tags") && el["tags"].contains("natural") && el["tags"]["natural"] == "coastline" && el.contains("geometry")) {
                        const auto& geom = el["geometry"];
                        for (size_t i = 0; i + 1 < geom.size(); ++i) {
                            Vec2d p0 = proj.latLonToWorldXZ(geom[i].value("lat", 0.0), geom[i].value("lon", 0.0));
                            Vec2d p1 = proj.latLonToWorldXZ(geom[i + 1].value("lat", 0.0), geom[i + 1].value("lon", 0.0));
                            double len = std::hypot(p1.x - p0.x, p1.z - p0.z);
                            int steps = std::max(1, (int)(len / 10.0));
                            for (int s = 0; s <= steps; ++s) {
                                double t = (double)s / steps;
                                ensureDryCellAt((float)(p0.x + (p1.x - p0.x) * t), (float)(p0.z + (p1.z - p0.z) * t));
                            }
                        }
                    }
                }

                std::unordered_set<std::string> seenStationNames;
                for (const auto& el : j["elements"]) {
                    if (el.value("type", "") != "node" || !el.contains("tags")) continue;
                    const auto& tags = el["tags"];
                    std::string rTag = tags.value("railway", "");
                    if (rTag != "station" && rTag != "halt") continue;
                    std::string net = tags.value("network", "");
                    std::string sub = tags.value("subway", "");
                    if (sub == "yes" && net.find("İZBAN") == std::string::npos && net.find("IZBAN") == std::string::npos) continue;

                    std::string rawName = tags.value("name", "");
                    if (rawName.empty()) continue;
                    std::string cleanName = sanitizeTurkishUpper(rawName);
                    if (cleanName.size() < 3 || !seenStationNames.insert(cleanName).second) continue;

                    Vec2d xz = proj.latLonToWorldXZ(el.value("lat", 0.0), el.value("lon", 0.0));
                    StationRecord sr;
                    if (cleanName == "EGEKENT 2") std::strncpy(sr.code, "EGK2", sizeof(sr.code) - 1);
                    else if (cleanName == "EGEKENT") std::strncpy(sr.code, "EGK1", sizeof(sr.code) - 1);
                    else std::strncpy(sr.code, cleanName.substr(0, std::min((size_t)4, cleanName.size())).c_str(), sizeof(sr.code) - 1);

                    std::strncpy(sr.name, cleanName.c_str(), sizeof(sr.name) - 1);
                    sr.x = (float)xz.x; sr.z = (float)xz.z;
                    bool wat = false;
                    sr.y = world.sampleHeightAt(sr.x, sr.z, wat);
                    sr.yawRad = 0.0f;
                    world.stations.push_back(sr);
                }
            }
        }
    }

    // Flood-Fill Deniz Maskesi
    {
        std::vector<uint8_t> conn(NX * NZ, 0);
        std::queue<uint32_t> q;
        for (uint32_t z = 0; z < NZ; ++z) {
            uint32_t idx = z * NX;
            if (world.demWaterMask[idx]) { conn[idx] = 1; q.push(idx); }
        }
        const int dx[4] = {-1, 1, 0, 0}, dz[4] = {0, 0, -1, 1};
        while (!q.empty()) {
            uint32_t c = q.front(); q.pop();
            int cx = (int)(c % NX), cz = (int)(c / NX);
            for (int k = 0; k < 4; ++k) {
                int nx = cx + dx[k], nz = cz + dz[k];
                if (nx < 0 || nx >= (int)NX || nz < 0 || nz >= (int)NZ) continue;
                uint32_t nIdx = (uint32_t)nz * NX + (uint32_t)nx;
                if (!conn[nIdx] && world.demWaterMask[nIdx]) { conn[nIdx] = 1; q.push(nIdx); }
            }
        }
        for (uint32_t i = 0; i < NX * NZ; ++i) {
            if (world.demWaterMask[i] && !conn[i]) {
                world.demWaterMask[i] = 0;
                world.demHeights[i]   = 2.8f;
            }
        }
    }

    std::sort(world.stations.begin(), world.stations.end(), [](const StationRecord& a, const StationRecord& b) {
        return a.z < b.z;
    });
    for (auto& st : world.stations) {
        snapStationToNearestRail(st, world);
    }

    curl_easy_cleanup(curl);
    curl_global_cleanup();

    std::string outPath = "../data/compiled/izmir_world.izmb";
    world.saveToBinary(outPath);
    std::cout << "\n============================================================================\n";
    std::cout << "  [v13.0 DERLEMESI TAMAMLANDI]\n";
    std::cout << "  * Gercek Uydu Binasi : " << world.buildings.size() << " adet (Eksiksiz!)\n";
    std::cout << "  * Ulasim Agi (Yol)   : " << world.ways.size() << " hat (Alt gecitler & sokaklar tam!)\n";
    std::cout << "============================================================================\n";
    return 0;
}
