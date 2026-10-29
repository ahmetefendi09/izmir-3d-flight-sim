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

struct WayVertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
    uint8_t r, g, b, a;
};

struct WaySectorVBO {
    GLuint vboArterial    = 0; // Otoyol, Bulvar, Cadde, IZBAN & Istasyonlar (13 km menzil)
    uint32_t countArterial = 0;
    GLuint vboResidential = 0; // Ara sokaklar (4.2 km menzil -> Devasa FPS Artisi!)
    uint32_t countResidential = 0;
    float centerX = 0.0f;
    float centerZ = 0.0f;
    float radius  = 1600.0f;
};

class WayAndRailRenderer {
private:
    static constexpr int GRID_X = 16;
    static constexpr int GRID_Z = 24;
    WaySectorVBO m_sectors[GRID_Z][GRID_X];
    ShaderProgram m_shader;

    static void pushTri(std::vector<WayVertex>& buf,
                        const Vec3f& p0, const Vec3f& p1, const Vec3f& p2,
                        float u0, float v0, float u1, float v1, float u2, float v2,
                        uint8_t r, uint8_t g, uint8_t b, uint8_t catByte) {
        buf.push_back({p0.x, p0.y, p0.z, 0.0f, 1.0f, 0.0f, u0, v0, r, g, b, catByte});
        buf.push_back({p1.x, p1.y, p1.z, 0.0f, 1.0f, 0.0f, u1, v1, r, g, b, catByte});
        buf.push_back({p2.x, p2.y, p2.z, 0.0f, 1.0f, 0.0f, u2, v2, r, g, b, catByte});
    }

    static void pushRibbonQuad(std::vector<WayVertex>& buf,
                               const Vec3f& l0, const Vec3f& r0, const Vec3f& r1, const Vec3f& l1,
                               float v0, float v1, uint8_t r, uint8_t g, uint8_t b, uint8_t catByte) {
        pushTri(buf, l0, r0, r1, -1.0f, v0,  1.0f, v0,  1.0f, v1, r, g, b, catByte);
        pushTri(buf, l0, r1, l1, -1.0f, v0,  1.0f, v1, -1.0f, v1, r, g, b, catByte);
    }

    static void pushJointDisc(std::vector<WayVertex>& buf, const Vec3f& center, float radius,
                              uint8_t r, uint8_t g, uint8_t b, uint8_t catByte) {
        const int SEGS = 8;
        for (int i = 0; i < SEGS; ++i) {
            float a0 = (i / (float)SEGS) * 2.0f * (float)GeoProjection::PI;
            float a1 = ((i + 1) / (float)SEGS) * 2.0f * (float)GeoProjection::PI;
            Vec3f p0(center.x + std::cos(a0) * radius, center.y, center.z + std::sin(a0) * radius);
            Vec3f p1(center.x + std::cos(a1) * radius, center.y, center.z + std::sin(a1) * radius);
            pushTri(buf, center, p0, p1, 0.0f, -1.0f, 0.5f, -1.0f, 0.5f, -1.0f, r, g, b, catByte);
        }
    }

    static void pushOrientedBox(std::vector<WayVertex>& buf,
                                float cx, float y0, float cz,
                                float halfW, float halfL, float h,
                                float cosA, float sinA,
                                uint8_t r, uint8_t g, uint8_t b) {
        float lx[4] = { -halfW,  halfW,  halfW, -halfW };
        float lz[4] = { -halfL, -halfL,  halfL,  halfL };
        Vec3f pB[4], pT[4];
        for (int k = 0; k < 4; ++k) {
            float wx = cx + lx[k] * cosA + lz[k] * sinA;
            float wz = cz - lx[k] * sinA + lz[k] * cosA;
            pB[k] = Vec3f(wx, y0,     wz);
            pT[k] = Vec3f(wx, y0 + h, wz);
        }
        uint8_t sR = (uint8_t)(r * 0.82f), sG = (uint8_t)(g * 0.82f), sB = (uint8_t)(b * 0.82f);
        for (int k = 0; k < 4; ++k) {
            int nxt = (k + 1) & 3;
            pushRibbonQuad(buf, pB[k], pB[nxt], pT[nxt], pT[k], -1.0f, -1.0f, sR, sG, sB, 128);
        }
        pushRibbonQuad(buf, pT[0], pT[1], pT[2], pT[3], -1.0f, -1.0f, r, g, b, 128);
    }

public:
    bool initFromWorldData(const IzmbWorldData& world, const std::string& shaderDir = "../shaders") {
        if (!m_shader.loadFromFiles(shaderDir + "/road_rail.vert", shaderDir + "/road_rail.frag")) {
            return false;
        }

        float minX = world.header.minX, maxX = world.header.maxX;
        float minZ = world.header.minZ, maxZ = world.header.maxZ;
        float spanX = maxX - minX, spanZ = maxZ - minZ;
        float cellW = spanX / GRID_X, cellZ = spanZ / GRID_Z;

        std::vector<WayVertex> artVerts[GRID_Z][GRID_X];
        std::vector<WayVertex> resVerts[GRID_Z][GRID_X];

        for (int gz = 0; gz < GRID_Z; ++gz) {
            for (int gx = 0; gx < GRID_X; ++gx) {
                m_sectors[gz][gx].centerX = minX + (gx + 0.5f) * cellW;
                m_sectors[gz][gx].centerZ = minZ + (gz + 0.5f) * cellZ;
                m_sectors[gz][gx].radius  = std::hypot(cellW, cellZ) * 0.65f;
            }
        }

        for (const auto& w : world.ways) {
            if (w.centerline.size() < 2) continue;

            float halfW = 2.1f;
            float baseLift = 0.26f;
            uint8_t aspR = 62, aspG = 65, aspB = 72;
            uint8_t catByte = 76; // cat 3: Residential
            bool isArterial = false;

            if (w.category == WayCategory::IzbanRailway) {
                baseLift   = 0.38f;
                halfW      = 2.5f;
                catByte    = 102; // cat 4
                isArterial = true;
            } else if (w.category == WayCategory::Motorway) {
                baseLift   = 0.34f;
                halfW      = 4.2f;
                aspR = 46; aspG = 49; aspB = 56;
                catByte    = 0;
                isArterial = true;
            } else if (w.category == WayCategory::Primary) {
                baseLift   = 0.30f;
                halfW      = 3.5f;
                aspR = 52; aspG = 55; aspB = 62;
                catByte    = 25;
                isArterial = true;
            } else if (w.category == WayCategory::Secondary) {
                baseLift   = 0.28f;
                halfW      = 2.8f;
                aspR = 58; aspG = 61; aspB = 68;
                catByte    = 51;
                isArterial = true;
            }

            float totalWayLen = 0.0f;
            for (size_t i = 0; i + 1 < w.centerline.size(); ++i) {
                totalWayLen += std::hypot(w.centerline[i + 1].x - w.centerline[i].x,
                                          w.centerline[i + 1].z - w.centerline[i].z);
            }

            float distAccum = 0.0f;
            for (size_t i = 0; i + 1 < w.centerline.size(); ++i) {
                Vec3f rawA = w.centerline[i];
                Vec3f rawB = w.centerline[i + 1];
                float dx = rawB.x - rawA.x;
                float dz = rawB.z - rawA.z;
                float segLen = std::hypot(dx, dz);
                if (segLen < 0.5f || segLen > 750.0f) continue;

                int sx = std::clamp((int)(((rawA.x - minX) / spanX) * GRID_X), 0, GRID_X - 1);
                int sz = std::clamp((int)(((rawA.z - minZ) / spanZ) * GRID_Z), 0, GRID_Z - 1);
                auto& targetBuf = isArterial ? artVerts[sz][sx] : resVerts[sz][sx];

                float nx = -dz / segLen;
                float nz =  dx / segLen;

                int nSub = std::clamp((int)std::ceil(segLen / 12.0f), 1, 70);
                for (int s = 0; s < nSub; ++s) {
                    float t0 = (float)s / (float)nSub;
                    float t1 = (float)(s + 1) / (float)nSub;
                    Vec3f p0(rawA.x + dx * t0, 0.0f, rawA.z + dz * t0);
                    Vec3f p1(rawA.x + dx * t1, 0.0f, rawA.z + dz * t1);

                    float d0 = distAccum + segLen * t0;
                    float d1 = distAccum + segLen * t1;

                    auto sampleSafeY = [&](float cx, float cz, float dAlong) -> float {
                        bool wC = false, wL = false, wR = false;
                        float yC = world.sampleHeightAt(cx, cz, wC);
                        float yL = world.sampleHeightAt(cx - nx * halfW, cz - nz * halfW, wL);
                        float yR = world.sampleHeightAt(cx + nx * halfW, cz + nz * halfW, wR);
                        float gy = std::max({yC, yL, yR});
                        float brg = 0.0f;
                        if (w.isBridge) {
                            float ramp = std::min({1.0f, dAlong / 24.0f, (totalWayLen - dAlong) / 24.0f});
                            brg = std::max(0.0f, ramp) * 4.6f;
                        }
                        return gy + baseLift + brg;
                    };

                    p0.y = sampleSafeY(p0.x, p0.z, d0);
                    p1.y = sampleSafeY(p1.x, p1.z, d1);

                    Vec3f sl0(p0.x - nx * halfW, p0.y, p0.z - nz * halfW);
                    Vec3f sr0(p0.x + nx * halfW, p0.y, p0.z + nz * halfW);
                    Vec3f sr1(p1.x + nx * halfW, p1.y, p1.z + nz * halfW);
                    Vec3f sl1(p1.x - nx * halfW, p1.y, p1.z - nz * halfW);

                    pushRibbonQuad(targetBuf, sl0, sr0, sr1, sl1, d0, d1, aspR, aspG, aspB, catByte);
                    if (s == 0) {
                        pushJointDisc(targetBuf, p0, halfW, aspR, aspG, aspB, catByte);
                    }
                    if (i + 2 == w.centerline.size() && s + 1 == nSub) {
                        pushJointDisc(targetBuf, p1, halfW, aspR, aspG, aspB, catByte);
                    }
                }
                distAccum += segLen;
            }
        }

        // 29 GERCEK OSM IZBAN ISTASYONU
        for (const auto& st : world.stations) {
            int sx = std::clamp((int)(((st.x - minX) / spanX) * GRID_X), 0, GRID_X - 1);
            int sz = std::clamp((int)(((st.z - minZ) / spanZ) * GRID_Z), 0, GRID_Z - 1);
            auto& buf = artVerts[sz][sx];

            bool wat = false;
            float gy = world.sampleHeightAt(st.x, st.z, wat);
            float cosA = std::cos(st.yawRad);
            float sinA = std::sin(st.yawRad);

            for (int side = -1; side <= 1; side += 2) {
                float offX = side * 6.8f;
                float px = st.x + offX * cosA;
                float pz = st.z - offX * sinA;
                pushOrientedBox(buf, px, gy + 0.3f, pz, 2.6f, 62.0f, 1.35f, cosA, sinA, 215, 220, 228);
                float yx = st.x + (offX - side * 1.9f) * cosA;
                float yz = st.z - (offX - side * 1.9f) * sinA;
                pushOrientedBox(buf, yx, gy + 1.68f, yz, 0.30f, 60.0f, 0.08f, cosA, sinA, 250, 210, 30);
                pushOrientedBox(buf, px, gy + 5.6f, pz, 3.0f, 52.0f, 0.65f, cosA, sinA, 15, 118, 225);
            }

            pushOrientedBox(buf, st.x, gy + 6.4f, st.z, 11.0f, 9.5f, 4.4f, cosA, sinA, 242, 246, 252);
            pushOrientedBox(buf, st.x, gy + 10.8f, st.z, 11.8f, 10.2f, 0.9f, cosA, sinA, 245, 110, 25);
        }

        for (int gz = 0; gz < GRID_Z; ++gz) {
            for (int gx = 0; gx < GRID_X; ++gx) {
                auto& sec = m_sectors[gz][gx];
                auto& vA = artVerts[gz][gx];
                auto& vR = resVerts[gz][gx];
                sec.countArterial    = (uint32_t)vA.size();
                sec.countResidential = (uint32_t)vR.size();
                if (!vA.empty()) {
                    glGenBuffers(1, &sec.vboArterial);
                    glBindBuffer(GL_ARRAY_BUFFER, sec.vboArterial);
                    glBufferData(GL_ARRAY_BUFFER, vA.size() * sizeof(WayVertex), vA.data(), GL_STATIC_DRAW);
                }
                if (!vR.empty()) {
                    glGenBuffers(1, &sec.vboResidential);
                    glBindBuffer(GL_ARRAY_BUFFER, sec.vboResidential);
                    glBufferData(GL_ARRAY_BUFFER, vR.size() * sizeof(WayVertex), vR.data(), GL_STATIC_DRAW);
                }
            }
        }
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        return true;
    }

    void render(bool nightMode, const Vec3f& sunDir, const Vec3f& camPos, const Vec3f& camFwd, const Vec3f& fogCol) {
        m_shader.use();
        m_shader.setFloat("uNightMode", nightMode ? 1.0f : 0.0f);
        m_shader.setVec3("uSunDir", sunDir);
        m_shader.setVec3("uCameraPos", camPos);
        m_shader.setVec3("uFogColor", fogCol);

        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-3.5f, -6.0f);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        float fwdX = camFwd.x, fwdZ = camFwd.z;
        float fwdLen = std::hypot(fwdX, fwdZ);
        if (fwdLen > 1e-4f) { fwdX /= fwdLen; fwdZ /= fwdLen; }

        for (int gz = 0; gz < GRID_Z; ++gz) {
            for (int gx = 0; gx < GRID_X; ++gx) {
                const auto& sec = m_sectors[gz][gx];
                float dx = sec.centerX - camPos.x;
                float dz = sec.centerZ - camPos.z;
                float dist = std::hypot(dx, dz);
                if (dist > 9500.0f + sec.radius) continue;
                if (dist > sec.radius * 1.4f) {
                    if ((dx * fwdX + dz * fwdZ) / dist < -0.30f) continue;
                }

                // 1. Ana Bulvarlar, Otoyollar ve IZBAN Raylari (13 km menzil)
                if (sec.vboArterial && sec.countArterial > 0) {
                    glBindBuffer(GL_ARRAY_BUFFER, sec.vboArterial);
                    glVertexPointer(3, GL_FLOAT, sizeof(WayVertex), reinterpret_cast<const void*>(offsetof(WayVertex, x)));
                    glNormalPointer(GL_FLOAT, sizeof(WayVertex), reinterpret_cast<const void*>(offsetof(WayVertex, nx)));
                    glTexCoordPointer(2, GL_FLOAT, sizeof(WayVertex), reinterpret_cast<const void*>(offsetof(WayVertex, u)));
                    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(WayVertex), reinterpret_cast<const void*>(offsetof(WayVertex, r)));
                    glDrawArrays(GL_TRIANGLES, 0, sec.countArterial);
                }

                // 2. Ara Sokaklar (Sadece 4.5 km yakin cevrede cizilir -> 4x FPS Artisi!)
                if (dist < 3200.0f + sec.radius && sec.vboResidential && sec.countResidential > 0) {
                    glBindBuffer(GL_ARRAY_BUFFER, sec.vboResidential);
                    glVertexPointer(3, GL_FLOAT, sizeof(WayVertex), reinterpret_cast<const void*>(offsetof(WayVertex, x)));
                    glNormalPointer(GL_FLOAT, sizeof(WayVertex), reinterpret_cast<const void*>(offsetof(WayVertex, nx)));
                    glTexCoordPointer(2, GL_FLOAT, sizeof(WayVertex), reinterpret_cast<const void*>(offsetof(WayVertex, u)));
                    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(WayVertex), reinterpret_cast<const void*>(offsetof(WayVertex, r)));
                    glDrawArrays(GL_TRIANGLES, 0, sec.countResidential);
                }
            }
        }

        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glDisable(GL_POLYGON_OFFSET_FILL);
        ShaderProgram::unbind();
    }
};

} // namespace izmir
