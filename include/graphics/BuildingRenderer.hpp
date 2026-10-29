#pragma once
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "../core/GeoProjection.hpp"
#include "../core/IzmbFormat.hpp"
#include "../core/Mat4.hpp"
#include "ShaderProgram.hpp"

namespace izmir {

struct BldgVertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
    uint8_t r, g, b, a;
};

struct BuildingSectorVBO {
    GLuint vboHigh = 0; // Gokdelenler, Yuksek Siteler, Camiler & Buyuk Kompleksler (11.5 km)
    uint32_t countHigh = 0;
    GLuint vboMid  = 0; // Orta Boy Apartmanlar (5.2 km)
    uint32_t countMid  = 0;
    GLuint vboLow  = 0; // Kucuk 1-3 Katli Evler (2.8 km)
    uint32_t countLow  = 0;
    float centerX = 0.0f;
    float centerZ = 0.0f;
    float radius  = 1600.0f;
};

struct CollisionBuildingBox {
    float minX, maxX;
    float minZ, maxZ;
    float topY;
    uint32_t worldBldgIndex; // UINT32_MAX ise Ozel Simge (Saat Kulesi / Folkart / Mistral)
};

class BuildingRenderer {
private:
    static constexpr int GRID_X = 16;
    static constexpr int GRID_Z = 24;
    BuildingSectorVBO m_sectors[GRID_Z][GRID_X];
    ShaderProgram m_shader;

    // 128 x 192 Hizli Uzamsal Bina Carpisma Izgarasi (O(1) Sifir FPS Kaybiyla Carpisma Testi!)
    static constexpr int COL_NX = 128;
    static constexpr int COL_NZ = 192;
    float m_minX = -15000.0f, m_spanX = 27000.0f;
    float m_minZ = -23000.0f, m_spanZ = 41000.0f;
    std::vector<CollisionBuildingBox> m_colBoxes;
    std::vector<uint32_t> m_colGrid[COL_NZ][COL_NX];

    void registerCollisionBox(const CollisionBuildingBox& cb) {
        uint32_t idx = (uint32_t)m_colBoxes.size();
        m_colBoxes.push_back(cb);

        int gx0 = std::clamp((int)(((cb.minX - m_minX) / m_spanX) * (COL_NX - 1)), 0, COL_NX - 1);
        int gx1 = std::clamp((int)(((cb.maxX - m_minX) / m_spanX) * (COL_NX - 1)), 0, COL_NX - 1);
        int gz0 = std::clamp((int)(((cb.minZ - m_minZ) / m_spanZ) * (COL_NZ - 1)), 0, COL_NZ - 1);
        int gz1 = std::clamp((int)(((cb.maxZ - m_minZ) / m_spanZ) * (COL_NZ - 1)), 0, COL_NZ - 1);

        for (int gz = gz0; gz <= gz1; ++gz) {
            for (int gx = gx0; gx <= gx1; ++gx) {
                m_colGrid[gz][gx].push_back(idx);
            }
        }
    }

    static bool pointInPolygonXZ(float px, float pz, const std::vector<Vec3f>& poly, float radius,
                                 float& outClosestX, float& outClosestZ) {
        bool inside = false;
        int n = (int)poly.size();
        float bestDistSq = 1e18f;
        outClosestX = px;
        outClosestZ = pz;

        for (int i = 0, j = n - 1; i < n; j = i++) {
            const Vec3f& vi = poly[i];
            const Vec3f& vj = poly[j];

            if (((vi.z > pz) != (vj.z > pz)) &&
                (px < (vj.x - vi.x) * (pz - vi.z) / (vj.z - vi.z + 1e-7f) + vi.x)) {
                inside = !inside;
            }

            float dx = vj.x - vi.x, dz = vj.z - vi.z;
            float lSq = dx * dx + dz * dz;
            float t = (lSq > 1e-4f) ? std::clamp(((px - vi.x) * dx + (pz - vi.z) * dz) / lSq, 0.0f, 1.0f) : 0.0f;
            float cx = vi.x + t * dx, cz = vi.z + t * dz;
            float dSq = (px - cx) * (px - cx) + (pz - cz) * (pz - cz);
            if (dSq < bestDistSq) {
                bestDistSq = dSq;
                outClosestX = cx;
                outClosestZ = cz;
            }
        }
        return inside || (bestDistSq <= radius * radius);
    }

    static void pushTri(std::vector<BldgVertex>& buf,
                        const Vec3f& p0, const Vec3f& p1, const Vec3f& p2,
                        const Vec3f& n, float u, float v,
                        uint8_t r, uint8_t g, uint8_t b, uint8_t ao = 255) {
        buf.push_back({p0.x, p0.y, p0.z, n.x, n.y, n.z, u, v, r, g, b, ao});
        buf.push_back({p1.x, p1.y, p1.z, n.x, n.y, n.z, u, v, r, g, b, ao});
        buf.push_back({p2.x, p2.y, p2.z, n.x, n.y, n.z, u, v, r, g, b, ao});
    }

    static void pushWallQuad(std::vector<BldgVertex>& buf,
                             const Vec3f& b0, const Vec3f& b1, const Vec3f& t1, const Vec3f& t0,
                             const Vec3f& n, float uSpan, float vSpan,
                             uint8_t r, uint8_t g, uint8_t b, uint8_t botAO = 140) {
        BldgVertex v0{b0.x, b0.y, b0.z, n.x, n.y, n.z, 0.0f,  0.0f,  r, g, b, botAO};
        BldgVertex v1{b1.x, b1.y, b1.z, n.x, n.y, n.z, uSpan, 0.0f,  r, g, b, botAO};
        BldgVertex v2{t1.x, t1.y, t1.z, n.x, n.y, n.z, uSpan, vSpan, r, g, b, 255};
        BldgVertex v3{t0.x, t0.y, t0.z, n.x, n.y, n.z, 0.0f,  vSpan, r, g, b, 255};
        buf.push_back(v0); buf.push_back(v1); buf.push_back(v2);
        buf.push_back(v0); buf.push_back(v2); buf.push_back(v3);
    }

    static void pushSolidQuad(std::vector<BldgVertex>& buf,
                              const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, const Vec3f& p3,
                              const Vec3f& n, float uTag, uint8_t r, uint8_t g, uint8_t b, uint8_t ao = 255) {
        BldgVertex v0{p0.x, p0.y, p0.z, n.x, n.y, n.z, uTag, 0.0f, r, g, b, ao};
        BldgVertex v1{p1.x, p1.y, p1.z, n.x, n.y, n.z, uTag, 0.0f, r, g, b, ao};
        BldgVertex v2{p2.x, p2.y, p2.z, n.x, n.y, n.z, uTag, 0.0f, r, g, b, ao};
        BldgVertex v3{p3.x, p3.y, p3.z, n.x, n.y, n.z, uTag, 0.0f, r, g, b, ao};
        buf.push_back(v0); buf.push_back(v1); buf.push_back(v2);
        buf.push_back(v0); buf.push_back(v2); buf.push_back(v3);
    }

    static void bakeCylinder(std::vector<BldgVertex>& buf, float cx, float y0, float y1, float cz,
                             float rBot, float rTop, int segs, uint8_t r, uint8_t g, uint8_t b) {
        for (int i = 0; i < segs; ++i) {
            float a0 = (i / (float)segs) * 2.0f * (float)GeoProjection::PI;
            float a1 = ((i + 1) / (float)segs) * 2.0f * (float)GeoProjection::PI;
            float c0 = std::cos(a0), s0 = std::sin(a0);
            float c1 = std::cos(a1), s1 = std::sin(a1);
            pushSolidQuad(buf,
                          Vec3f(cx + c0 * rBot, y0, cz + s0 * rBot),
                          Vec3f(cx + c1 * rBot, y0, cz + s1 * rBot),
                          Vec3f(cx + c1 * rTop, y1, cz + s1 * rTop),
                          Vec3f(cx + c0 * rTop, y1, cz + s0 * rTop),
                          Vec3f(c0, 0.15f, s0), -1.0f, r, g, b, 235);
        }
    }

    static void bakeDome(std::vector<BldgVertex>& buf, float cx, float yBase, float cz,
                         float radius, float height, int segsU, int segsV,
                         uint8_t r, uint8_t g, uint8_t b) {
        for (int iv = 0; iv < segsV; ++iv) {
            float phi0 = (iv / (float)segsV) * (float)(GeoProjection::PI * 0.5);
            float phi1 = ((iv + 1) / (float)segsV) * (float)(GeoProjection::PI * 0.5);
            float y0 = yBase + std::sin(phi0) * height;
            float y1 = yBase + std::sin(phi1) * height;
            float rad0 = std::cos(phi0) * radius;
            float rad1 = std::cos(phi1) * radius;
            float ny0 = std::sin(phi0);

            for (int iu = 0; iu < segsU; ++iu) {
                float th0 = (iu / (float)segsU) * 2.0f * (float)GeoProjection::PI;
                float th1 = ((iu + 1) / (float)segsU) * 2.0f * (float)GeoProjection::PI;
                float c0 = std::cos(th0), s0 = std::sin(th0);
                float c1 = std::cos(th1), s1 = std::sin(th1);
                pushSolidQuad(buf,
                              Vec3f(cx + c0 * rad0, y0, cz + s0 * rad0),
                              Vec3f(cx + c1 * rad0, y0, cz + s1 * rad0),
                              Vec3f(cx + c1 * rad1, y1, cz + s1 * rad1),
                              Vec3f(cx + c0 * rad1, y1, cz + s0 * rad1),
                              Vec3f(c0, ny0, s0), -1.0f, r, g, b, 255);
            }
        }
    }

    static void bakeArchitecturalBox(std::vector<BldgVertex>& buf,
                                     float cx, float y0, float cz,
                                     float halfW, float halfD, float h,
                                     float cosA, float sinA,
                                     uint8_t r, uint8_t g, uint8_t b) {
        float lx[4] = { -halfW,  halfW,  halfW, -halfW };
        float lz[4] = {  halfD,  halfD, -halfD, -halfD };
        Vec3f pB[4], pT[4];
        for (int k = 0; k < 4; ++k) {
            float wx = cx + lx[k] * cosA - lz[k] * sinA;
            float wz = cz + lx[k] * sinA + lz[k] * cosA;
            pB[k] = Vec3f(wx, y0,     wz);
            pT[k] = Vec3f(wx, y0 + h, wz);
        }
        float vFloors = std::max(1.0f, std::round(h / 3.1f));
        for (int k = 0; k < 4; ++k) {
            int nxt = (k + 1) & 3;
            float edgeLen = std::hypot(pB[nxt].x - pB[k].x, pB[nxt].z - pB[k].z);
            float uCols = std::max(1.0f, std::round(edgeLen / 3.4f));
            Vec3f n = normalize3(Vec3f(-(pB[nxt].z - pB[k].z), 0.0f, pB[nxt].x - pB[k].x));
            pushWallQuad(buf, pB[k], pB[nxt], pT[nxt], pT[k], n, uCols, vFloors, r, g, b, 145);
        }
        pushSolidQuad(buf, pT[0], pT[1], pT[2], pT[3], Vec3f(0, 1, 0), -1.0f,
                      (uint8_t)(r * 0.85f), (uint8_t)(g * 0.85f), (uint8_t)(b * 0.85f), 245);
    }

    static bool isStrictlyConvexQuad(const std::vector<Vec3f>& pts) {
        if (pts.size() != 4) return false;
        float firstSign = 0.0f;
        for (int i = 0; i < 4; ++i) {
            const Vec3f& a = pts[i];
            const Vec3f& b = pts[(i + 1) & 3];
            const Vec3f& c = pts[(i + 2) & 3];
            float cross = (b.x - a.x) * (c.z - b.z) - (b.z - a.z) * (c.x - b.x);
            if (std::fabs(cross) < 1.0f) return false;
            if (i == 0) firstSign = cross;
            else if (cross * firstSign <= 0.0f) return false;
        }
        return true;
    }

public:
    // ========================================================================
    // GERCEK ZAMANLI 3D BINA CARPISMA TESTI (O(1) Spatial Hash + 2D Poligon)
    // ========================================================================
    bool checkBuildingCollision(const IzmbWorldData& world, const Vec3f& pos, float radius,
                                float& outHitRoofY, Vec3f& outPushNormal) const {
        int gx = (int)(((pos.x - m_minX) / m_spanX) * (COL_NX - 1));
        int gz = (int)(((pos.z - m_minZ) / m_spanZ) * (COL_NZ - 1));
        if (gx < 0 || gx >= COL_NX || gz < 0 || gz >= COL_NZ) return false;

        for (int dz = -1; dz <= 1; ++dz) {
            int nz = gz + dz;
            if (nz < 0 || nz >= COL_NZ) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                int nx = gx + dx;
                if (nx < 0 || nx >= COL_NX) continue;

                for (uint32_t boxIdx : m_colGrid[nz][nx]) {
                    const auto& cb = m_colBoxes[boxIdx];
                    if (pos.y - radius > cb.topY) continue;
                    if (pos.x + radius < cb.minX || pos.x - radius > cb.maxX ||
                        pos.z + radius < cb.minZ || pos.z - radius > cb.maxZ) {
                        continue;
                    }

                    // Ozel Simge Yapilar (Saat Kulesi, Folkart, Mistral, Ege Perla)
                    if (cb.worldBldgIndex == UINT32_MAX) {
                        outHitRoofY = cb.topY;
                        float cx = 0.5f * (cb.minX + cb.maxX);
                        float cz = 0.5f * (cb.minZ + cb.maxZ);
                        outPushNormal = normalize3(Vec3f(pos.x - cx, 0.2f, pos.z - cz));
                        return true;
                    }

                    // Gercek Bina Poligonu Testi
                    const auto& b = world.buildings[cb.worldBldgIndex];
                    float closeX = pos.x, closeZ = pos.z;
                    if (pointInPolygonXZ(pos.x, pos.z, b.footprintXZ, radius, closeX, closeZ)) {
                        outHitRoofY = cb.topY;
                        float pdx = pos.x - closeX, pdz = pos.z - closeZ;
                        if (std::hypot(pdx, pdz) < 0.05f) {
                            outPushNormal = Vec3f(0.0f, 1.0f, 0.0f);
                        } else {
                            outPushNormal = normalize3(Vec3f(pdx, 0.1f, pdz));
                        }
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool initFromWorldData(const IzmbWorldData& world, const std::string& shaderDir) {
        if (!m_shader.loadFromFiles(shaderDir + "/building.vert", shaderDir + "/building.frag")) {
            return false;
        }

        float minX = world.header.minX, maxX = world.header.maxX;
        float minZ = world.header.minZ, maxZ = world.header.maxZ;
        float spanX = maxX - minX, spanZ = maxZ - minZ;
        m_minX = minX; m_spanX = spanX;
        m_minZ = minZ; m_spanZ = spanZ;
        m_colBoxes.reserve(world.buildings.size() + 16);

        float cellW = spanX / GRID_X, cellZ = spanZ / GRID_Z;

        std::vector<BldgVertex> highVerts[GRID_Z][GRID_X];
        std::vector<BldgVertex> midVerts[GRID_Z][GRID_X];
        std::vector<BldgVertex> lowVerts[GRID_Z][GRID_X];

        for (int gz = 0; gz < GRID_Z; ++gz) {
            for (int gx = 0; gx < GRID_X; ++gx) {
                m_sectors[gz][gx].centerX = minX + (gx + 0.5f) * cellW;
                m_sectors[gz][gx].centerZ = minZ + (gz + 0.5f) * cellZ;
                m_sectors[gz][gx].radius  = std::hypot(cellW, cellZ) * 0.65f;
            }
        }

        for (uint32_t bIdx = 0; bIdx < (uint32_t)world.buildings.size(); ++bIdx) {
            const auto& b = world.buildings[bIdx];
            int K = (int)b.footprintXZ.size();
            if (K < 3 || b.roofIndices.size() < 3) continue;

            float cx = 0.0f, cz = 0.0f;
            float bMinX = 1e9f, bMaxX = -1e9f, bMinZ = 1e9f, bMaxZ = -1e9f;
            float minCornerY = 1e9f;
            float signedArea = 0.0f;

            for (int i = 0; i < K; ++i) {
                const Vec3f& p0 = b.footprintXZ[i];
                const Vec3f& p1 = b.footprintXZ[(i + 1) % K];
                signedArea += (p0.x * p1.z - p1.x * p0.z);
                cx += p0.x; cz += p0.z;
                bMinX = std::min(bMinX, p0.x); bMaxX = std::max(bMaxX, p0.x);
                bMinZ = std::min(bMinZ, p0.z); bMaxZ = std::max(bMaxZ, p0.z);

                bool wCorner = false;
                minCornerY = std::min(minCornerY, world.sampleHeightAt(p0.x, p0.z, wCorner));
            }
            cx /= K; cz /= K;

            float spanW = bMaxX - bMinX, spanD = bMaxZ - bMinZ;
            float maxSpan = std::max(spanW, spanD);
            float avgSpan = std::min(spanW, spanD);
            float areaM2  = 0.5f * std::fabs(signedArea);
            if (maxSpan > 600.0f || areaM2 < 16.0f) continue;

            bool watCenter = false;
            float centerGroundY = world.sampleHeightAt(cx, cz, watCenter);
            if (watCenter) continue;

            int sx = std::clamp((int)(((cx - minX) / spanX) * GRID_X), 0, GRID_X - 1);
            int sz = std::clamp((int)(((cz - minZ) / spanZ) * GRID_Z), 0, GRID_Z - 1);

            uint32_t seed = b.colorSeed;
            BuildingCategory cat = b.category;
            float bHeight = b.height;

            bool canPitchRoof = isStrictlyConvexQuad(b.footprintXZ) &&
                                (cat == BuildingCategory::StandardApartment) &&
                                (bHeight <= 21.0f) && (maxSpan <= 30.0f) &&
                                ((seed % 100) < 62);

            float yBase = minCornerY - 3.0f;
            float yTop  = centerGroundY + bHeight;
            float roofH = canPitchRoof ? std::clamp(avgSpan * 0.16f, 1.5f, 2.8f) : 0.0f;

            // Binayi Uzamsal Carpisma Izgarasina Kaydet!
            registerCollisionBox({bMinX, bMaxX, bMinZ, bMaxZ, yTop + roofH, bIdx});

            // 3-Kademeli Performans LOD Secimi:
            bool isHighTier = (bHeight >= 21.0f) || (areaM2 >= 360.0f) || (cat != BuildingCategory::StandardApartment);
            bool isMidTier  = !isHighTier && ((bHeight >= 11.5f) || (areaM2 >= 105.0f));
            auto& buf = isHighTier ? highVerts[sz][sx] : (isMidTier ? midVerts[sz][sx] : lowVerts[sz][sx]);

            float floors = std::max(1.0f, std::round(bHeight / 3.1f));

            uint8_t wR = 216, wG = 210, wB = 200;
            uint8_t rR = 148, rG = 78,  rB = 58;

            if (cat == BuildingCategory::GlassSkyscraper) {
                wR = (uint8_t)(118 + (seed % 32));
                wG = (uint8_t)(162 + (seed % 32));
                wB = (uint8_t)(218 + (seed % 30));
                rR = 142; rG = 148; rB = 156;
            } else if (cat == BuildingCategory::CommercialIndustrial) {
                wR = 182; wG = 190; wB = 202;
                rR = 154; rG = 160; rB = 168;
            } else {
                int wallPal = seed % 6;
                if (wallPal == 0)      { wR = 228; wG = 222; wB = 210; }
                else if (wallPal == 1) { wR = 208; wG = 212; wB = 216; }
                else if (wallPal == 2) { wR = 224; wG = 204; wB = 186; }
                else if (wallPal == 3) { wR = 196; wG = 192; wB = 184; }
                else if (wallPal == 4) { wR = 234; wG = 232; wB = 226; }
                else                   { wR = 212; wG = 198; wB = 178; }

                if (!canPitchRoof) {
                    int ter = 134 + (int)((seed >> 4) % 24);
                    rR = (uint8_t)ter; rG = (uint8_t)(ter - 2); rB = (uint8_t)(ter - 5);
                } else {
                    int roofPal = (seed >> 3) % 4;
                    if (roofPal == 0)      { rR = 156; rG = 78;  rB = 56; }
                    else if (roofPal == 1) { rR = 128; rG = 68;  rB = 52; }
                    else if (roofPal == 2) { rR = 162; rG = 92;  rB = 66; }
                    else                   { rR = 118; rG = 80;  rB = 64; }
                }
            }

            float orientSign = (signedArea >= 0.0f) ? 1.0f : -1.0f;

            for (int i = 0; i < K; ++i) {
                const Vec3f& p0 = b.footprintXZ[i];
                const Vec3f& p1 = b.footprintXZ[(i + 1) % K];
                float dx = p1.x - p0.x, dz = p1.z - p0.z;
                float edgeLen = std::hypot(dx, dz);
                if (edgeLen < 0.3f || edgeLen > 450.0f) continue;

                Vec3f n((dz / edgeLen) * orientSign, 0.0f, (-dx / edgeLen) * orientSign);
                float uCols = std::max(1.0f, std::round(edgeLen / 3.4f));

                if (orientSign > 0.0f) {
                    pushWallQuad(buf, Vec3f(p0.x, yBase, p0.z), Vec3f(p1.x, yBase, p1.z),
                                 Vec3f(p1.x, yTop,  p1.z), Vec3f(p0.x, yTop,  p0.z),
                                 n, uCols, floors, wR, wG, wB, 138);
                } else {
                    pushWallQuad(buf, Vec3f(p1.x, yBase, p1.z), Vec3f(p0.x, yBase, p0.z),
                                 Vec3f(p0.x, yTop,  p0.z), Vec3f(p1.x, yTop,  p1.z),
                                 n, uCols, floors, wR, wG, wB, 138);
                }
            }

            if (cat == BuildingCategory::Mosque) {
                for (size_t t = 0; t + 2 < b.roofIndices.size(); t += 3) {
                    const Vec3f& a = b.footprintXZ[b.roofIndices[t]];
                    const Vec3f& c = b.footprintXZ[b.roofIndices[t + 1]];
                    const Vec3f& d = b.footprintXZ[b.roofIndices[t + 2]];
                    pushTri(buf, Vec3f(a.x, yTop, a.z), Vec3f(c.x, yTop, c.z), Vec3f(d.x, yTop, d.z),
                            Vec3f(0, 1, 0), -1.0f, 0.0f, 128, 136, 148);
                }
                float domeR = std::clamp(avgSpan * 0.36f, 4.5f, 14.0f);
                bakeDome(buf, cx, yTop, cz, domeR, domeR * 0.85f, 10, 4, 118, 128, 142);
            } else if (canPitchRoof) {
                float yPeak = yTop + roofH;
                for (int i = 0; i < 4; ++i) {
                    const Vec3f& p0 = b.footprintXZ[i];
                    const Vec3f& p1 = b.footprintXZ[(i + 1) & 3];
                    Vec3f slopeN = normalize3(Vec3f((p1.z - p0.z) * orientSign, std::hypot(p1.x - p0.x, p1.z - p0.z) * 1.5f, -(p1.x - p0.x) * orientSign));
                    pushTri(buf, Vec3f(p0.x, yTop, p0.z), Vec3f(p1.x, yTop, p1.z), Vec3f(cx, yPeak, cz),
                            slopeN, -1.0f, 0.0f, rR, rG, rB);
                }
            } else {
                for (size_t t = 0; t + 2 < b.roofIndices.size(); t += 3) {
                    uint16_t i0 = b.roofIndices[t], i1 = b.roofIndices[t + 1], i2 = b.roofIndices[t + 2];
                    if (i0 >= K || i1 >= K || i2 >= K) continue;
                    const Vec3f& a = b.footprintXZ[i0];
                    const Vec3f& c = b.footprintXZ[i1];
                    const Vec3f& d = b.footprintXZ[i2];
                    pushTri(buf, Vec3f(a.x, yTop, a.z), Vec3f(c.x, yTop, c.z), Vec3f(d.x, yTop, d.z),
                            Vec3f(0, 1, 0), -1.0f, 0.0f, rR, rG, rB, 240);
                }
            }
        }

        // IZMIR SIMGELERI (Carpisma kutulariyla birlikte!)
        GeoProjection proj;
        auto addLandmark = [&](double lat, double lon, int type) {
            Vec2d xz = proj.latLonToWorldXZ(lat, lon);
            int sx = std::clamp((int)((((float)xz.x - minX) / spanX) * GRID_X), 0, GRID_X - 1);
            int sz = std::clamp((int)((((float)xz.z - minZ) / spanZ) * GRID_Z), 0, GRID_Z - 1);
            auto& buf = highVerts[sz][sx];
            bool wat = false;
            float gy = world.sampleHeightAt((float)xz.x, (float)xz.z, wat);
            float cx = (float)xz.x, cz = (float)xz.z;

            if (type == 0) {
                bakeCylinder(buf, cx, gy, gy + 2.5f, cz, 9.0f, 7.8f, 8, 240, 235, 225);
                bakeArchitecturalBox(buf, cx, gy + 2.5f, cz, 3.6f, 3.6f, 6.0f, 1, 0, 235, 226, 210);
                bakeCylinder(buf, cx, gy + 8.5f, gy + 19.5f, cz, 2.5f, 2.1f, 8, 238, 230, 214);
                bakeArchitecturalBox(buf, cx, gy + 19.5f, cz, 2.4f, 2.4f, 3.5f, 1, 0, 245, 240, 230);
                bakeDome(buf, cx, gy + 23.0f, cz, 2.1f, 2.4f, 10, 5, 195, 165, 95);
                registerCollisionBox({cx - 5.0f, cx + 5.0f, cz - 5.0f, cz + 5.0f, gy + 26.0f, UINT32_MAX});
            } else if (type == 1) {
                for (int t = -1; t <= 1; t += 2) {
                    float tx = cx + t * 32.0f, tz = cz + t * 12.0f;
                    for (int tier = 0; tier < 5; ++tier) {
                        float ty0 = gy + tier * 38.0f;
                        float rad = 18.0f - tier * 1.2f;
                        float ang = tier * 0.09f * t;
                        bakeArchitecturalBox(buf, tx, ty0, tz, rad, rad * 0.78f, 38.0f, std::cos(ang), std::sin(ang), 125, 180, 240);
                    }
                    registerCollisionBox({tx - 17.0f, tx + 17.0f, tz - 15.0f, tz + 15.0f, gy + 195.0f, UINT32_MAX});
                }
            } else if (type == 2) {
                bakeArchitecturalBox(buf, cx, gy, cz, 24.0f, 20.0f, 26.0f, 1, 0, 145, 190, 235);
                bakeArchitecturalBox(buf, cx, gy + 26.0f, cz, 16.5f, 14.0f, 185.0f, 0.92f, 0.38f, 120, 175, 238);
                registerCollisionBox({cx - 18.0f, cx + 18.0f, cz - 16.0f, cz + 16.0f, gy + 212.0f, UINT32_MAX});
            }
        };

        addLandmark(38.418898, 27.128677, 0);
        addLandmark(38.454200, 27.176400, 1);
        addLandmark(38.451800, 27.179500, 2);
        addLandmark(38.445200, 27.162500, 2);

        for (int gz = 0; gz < GRID_Z; ++gz) {
            for (int gx = 0; gx < GRID_X; ++gx) {
                auto& sec = m_sectors[gz][gx];
                auto& vH = highVerts[gz][gx];
                auto& vM = midVerts[gz][gx];
                auto& vL = lowVerts[gz][gx];
                sec.countHigh = (uint32_t)vH.size();
                sec.countMid  = (uint32_t)vM.size();
                sec.countLow  = (uint32_t)vL.size();
                if (!vH.empty()) {
                    glGenBuffers(1, &sec.vboHigh);
                    glBindBuffer(GL_ARRAY_BUFFER, sec.vboHigh);
                    glBufferData(GL_ARRAY_BUFFER, vH.size() * sizeof(BldgVertex), vH.data(), GL_STATIC_DRAW);
                }
                if (!vM.empty()) {
                    glGenBuffers(1, &sec.vboMid);
                    glBindBuffer(GL_ARRAY_BUFFER, sec.vboMid);
                    glBufferData(GL_ARRAY_BUFFER, vM.size() * sizeof(BldgVertex), vM.data(), GL_STATIC_DRAW);
                }
                if (!vL.empty()) {
                    glGenBuffers(1, &sec.vboLow);
                    glBindBuffer(GL_ARRAY_BUFFER, sec.vboLow);
                    glBufferData(GL_ARRAY_BUFFER, vL.size() * sizeof(BldgVertex), vL.data(), GL_STATIC_DRAW);
                }
            }
        }
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        std::cout << "[3-TIER LOD & CARPISMA MOTORU] " << m_colBoxes.size()
                  << " Bina Carpisma Kutusu (128x192 Spatial Hash) Hazir.\n";
        return true;
    }

    void render(bool nightMode, const Vec3f& sunDir, const Vec3f& camPos, const Vec3f& camFwd, const Vec3f& fogCol) {
        m_shader.use();
        m_shader.setFloat("uNightMode", nightMode ? 1.0f : 0.0f);
        m_shader.setVec3("uSunDir", sunDir);
        m_shader.setVec3("uCameraPos", camPos);
        m_shader.setVec3("uFogColor", fogCol);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        float fwdX = camFwd.x, fwdZ = camFwd.z;
        float fwdLen = std::hypot(fwdX, fwdZ);
        if (fwdLen > 1e-4f) { fwdX /= fwdLen; fwdZ /= fwdLen; }

        auto drawVbo = [](GLuint vbo, uint32_t cnt) {
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glVertexPointer(3, GL_FLOAT, sizeof(BldgVertex), reinterpret_cast<const void*>(offsetof(BldgVertex, x)));
            glNormalPointer(GL_FLOAT, sizeof(BldgVertex), reinterpret_cast<const void*>(offsetof(BldgVertex, nx)));
            glTexCoordPointer(2, GL_FLOAT, sizeof(BldgVertex), reinterpret_cast<const void*>(offsetof(BldgVertex, u)));
            glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(BldgVertex), reinterpret_cast<const void*>(offsetof(BldgVertex, r)));
            glDrawArrays(GL_TRIANGLES, 0, cnt);
        };

        for (int gz = 0; gz < GRID_Z; ++gz) {
            for (int gx = 0; gx < GRID_X; ++gx) {
                const auto& sec = m_sectors[gz][gx];
                float dx = sec.centerX - camPos.x;
                float dz = sec.centerZ - camPos.z;
                float dist = std::hypot(dx, dz);
                if (dist > 11000.0f + sec.radius) continue;

                if (dist > sec.radius * 1.4f) {
                    if ((dx * fwdX + dz * fwdZ) / dist < -0.32f) continue;
                }

                // LOD 1: Gokdelenler ve Buyuk Kompleksler (11 km)
                if (sec.vboHigh && sec.countHigh > 0) drawVbo(sec.vboHigh, sec.countHigh);
                // LOD 2: Standart Apartmanlar (5.4 km)
                if (dist < 5400.0f + sec.radius && sec.vboMid && sec.countMid > 0) drawVbo(sec.vboMid, sec.countMid);
                // LOD 3: Kucuk 1-3 Katli Evler (2.8 km -> Akici 60 FPS!)
                if (dist < 2800.0f + sec.radius && sec.vboLow && sec.countLow > 0) drawVbo(sec.vboLow, sec.countLow);
            }
        }

        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        ShaderProgram::unbind();
    }
};

} // namespace izmir
