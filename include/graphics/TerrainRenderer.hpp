#pragma once
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "../core/IzmbFormat.hpp"
#include "../core/Mat4.hpp"
#include "ShaderProgram.hpp"

#ifndef GL_TEXTURE_MAX_ANISOTROPY_EXT
#define GL_TEXTURE_MAX_ANISOTROPY_EXT     0x84FE
#endif
#ifndef GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT
#define GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT 0x84FF
#endif

namespace izmir {

struct TerrainVertex {
    float x, y, z;
    float nx, ny, nz;
    uint8_t r, g, b, a;
};

struct TerrainSectorEBO {
    GLuint ebo = 0;
    uint32_t indexCount = 0;
    float centerX = 0.0f;
    float centerZ = 0.0f;
    float radius  = 2600.0f;
};

class TerrainRenderer {
private:
    static constexpr int SEC_X = 8;
    static constexpr int SEC_Z = 12;
    GLuint m_vbo = 0;
    TerrainSectorEBO m_sectors[SEC_Z][SEC_X];

    GLuint m_noiseTex   = 0;
    GLuint m_roadMapTex = 0;
    ShaderProgram m_shader;

    static inline float hashFloat(int x, int y, uint32_t seed) {
        uint32_t h = (uint32_t)(x * 374761393U + y * 668265263U + seed * 1442695041U);
        h = (h ^ (h >> 13)) * 1274126177U;
        return (float)((h ^ (h >> 16)) & 0xFFFF) / 65535.0f;
    }

    static float tileableSmoothNoise(int x, int y, int period, int S, uint32_t seed) {
        int cell = std::max(1, S / period);
        int gx0 = (x / cell) % period;
        int gy0 = (y / cell) % period;
        int gx1 = (gx0 + 1) % period;
        int gy1 = (gy0 + 1) % period;

        float fx = (float)(x % cell) / (float)cell;
        float fy = (float)(y % cell) / (float)cell;
        float ux = fx * fx * (3.0f - 2.0f * fx);
        float uy = fy * fy * (3.0f - 2.0f * fy);

        float v00 = hashFloat(gx0, gy0, seed);
        float v10 = hashFloat(gx1, gy0, seed);
        float v01 = hashFloat(gx0, gy1, seed);
        float v11 = hashFloat(gx1, gy1, seed);

        float top = v00 * (1.0f - ux) + v10 * ux;
        float bot = v01 * (1.0f - ux) + v11 * ux;
        return top * (1.0f - uy) + bot * uy;
    }

    void buildOrganicNoiseTexture() {
        const int S = 512;
        std::vector<uint8_t> rgba(S * S * 4, 255);
        for (int y = 0; y < S; ++y) {
            for (int x = 0; x < S; ++x) {
                float c1 = tileableSmoothNoise(x, y, 8,  S, 101U) * 0.55f +
                           tileableSmoothNoise(x, y, 16, S, 102U) * 0.30f +
                           tileableSmoothNoise(x, y, 32, S, 103U) * 0.15f;
                float c2 = tileableSmoothNoise(x, y, 16, S, 201U) * 0.50f +
                           tileableSmoothNoise(x, y, 32, S, 202U) * 0.30f +
                           tileableSmoothNoise(x, y, 64, S, 203U) * 0.20f;
                float c3 = tileableSmoothNoise(x, y, 64,  S, 301U) * 0.50f +
                           tileableSmoothNoise(x, y, 128, S, 302U) * 0.30f +
                           hashFloat(x, y, 303U) * 0.20f;

                int idx = (y * S + x) * 4;
                rgba[idx]     = (uint8_t)std::clamp((int)(c1 * 255.0f), 0, 255);
                rgba[idx + 1] = (uint8_t)std::clamp((int)(c2 * 255.0f), 0, 255);
                rgba[idx + 2] = (uint8_t)std::clamp((int)(c3 * 255.0f), 0, 255);
                rgba[idx + 3] = 255;
            }
        }

        glGenTextures(1, &m_noiseTex);
        glBindTexture(GL_TEXTURE_2D, m_noiseTex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, S, S, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    }

    void buildWorldRoadAndUrbanTexture(const IzmbWorldData& world) {
        const int TW = 2048;
        const int TH = 2048;
        std::vector<uint8_t> tex((size_t)TW * (size_t)TH * 4, 0);

        float minX = world.header.minX, spanX = world.header.maxX - world.header.minX;
        float minZ = world.header.minZ, spanZ = world.header.maxZ - world.header.minZ;

        for (const auto& b : world.buildings) {
            if (b.footprintXZ.empty()) continue;
            int px = (int)(((b.footprintXZ[0].x - minX) / spanX) * (TW - 1));
            int py = (int)(((b.footprintXZ[0].z - minZ) / spanZ) * (TH - 1));
            for (int dy = -1; dy <= 1; ++dy) {
                int y = py + dy;
                if (y < 0 || y >= TH) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    int x = px + dx;
                    if (x < 0 || x >= TW) continue;
                    size_t idx = ((size_t)y * TW + x) * 4;
                    tex[idx + 1] = (uint8_t)std::min(225, (int)tex[idx + 1] + 115);
                }
            }
        }

        for (const auto& w : world.ways) {
            uint8_t rVal = 195, bVal = 40;
            bool thick = (w.category == WayCategory::Motorway || w.category == WayCategory::Primary || w.category == WayCategory::IzbanRailway);
            if (w.category == WayCategory::Motorway) { rVal = 255; bVal = 210; }
            else if (thick)                          { rVal = 240; bVal = 160; }
            else if (w.category == WayCategory::Secondary) { rVal = 220; bVal = 90; }

            for (size_t i = 0; i + 1 < w.centerline.size(); ++i) {
                const Vec3f& p0 = w.centerline[i];
                const Vec3f& p1 = w.centerline[i + 1];
                float fx0 = ((p0.x - minX) / spanX) * (TW - 1);
                float fy0 = ((p0.z - minZ) / spanZ) * (TH - 1);
                float fx1 = ((p1.x - minX) / spanX) * (TW - 1);
                float fy1 = ((p1.z - minZ) / spanZ) * (TH - 1);

                float dx = fx1 - fx0, dy = fy1 - fy0;
                int steps = std::clamp((int)std::ceil(std::max(std::fabs(dx), std::fabs(dy)) * 1.3f), 1, 500);
                for (int s = 0; s <= steps; ++s) {
                    float t = (float)s / (float)steps;
                    int cx = (int)(fx0 + dx * t);
                    int cy = (int)(fy0 + dy * t);
                    if (cx < 1 || cx >= TW - 1 || cy < 1 || cy >= TH - 1) continue;

                    size_t cIdx = ((size_t)cy * TW + cx) * 4;
                    tex[cIdx + 0] = std::max(tex[cIdx + 0], rVal);
                    tex[cIdx + 1] = std::max(tex[cIdx + 1], (uint8_t)165);
                    tex[cIdx + 2] = std::max(tex[cIdx + 2], bVal);

                    if (thick) {
                        uint8_t edgeR = 175;
                        tex[((size_t)cy * TW + (cx + 1)) * 4] = std::max(tex[((size_t)cy * TW + (cx + 1)) * 4], edgeR);
                        tex[((size_t)(cy + 1) * TW + cx) * 4] = std::max(tex[((size_t)(cy + 1) * TW + cx) * 4], edgeR);
                    }
                }
            }
        }

        glGenTextures(1, &m_roadMapTex);
        glBindTexture(GL_TEXTURE_2D, m_roadMapTex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, TW, TH, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.data());
    }

    static void smoothGridField(std::vector<float>& field, uint32_t NX, uint32_t NZ, int passes) {
        std::vector<float> tmp(NX * NZ, 0.0f);
        for (int p = 0; p < passes; ++p) {
            for (uint32_t z = 0; z < NZ; ++z) {
                uint32_t z0 = (z > 0) ? z - 1 : 0;
                uint32_t z1 = (z + 1 < NZ) ? z + 1 : NZ - 1;
                for (uint32_t x = 0; x < NX; ++x) {
                    uint32_t x0 = (x > 0) ? x - 1 : 0;
                    uint32_t x1 = (x + 1 < NX) ? x + 1 : NX - 1;
                    float sum = field[z * NX + x] * 4.0f +
                                field[z * NX + x0] * 2.0f + field[z * NX + x1] * 2.0f +
                                field[z0 * NX + x] * 2.0f + field[z1 * NX + x] * 2.0f +
                                field[z0 * NX + x0] + field[z0 * NX + x1] +
                                field[z1 * NX + x0] + field[z1 * NX + x1];
                    tmp[z * NX + x] = sum / 16.0f;
                }
            }
            field.swap(tmp);
        }
    }

public:
    bool initFromWorldData(IzmbWorldData& world, const std::string& shaderDir) {
        if (!m_shader.loadFromFiles(shaderDir + "/terrain_splat.vert", shaderDir + "/terrain_splat.frag")) {
            return false;
        }
        buildOrganicNoiseTexture();
        buildWorldRoadAndUrbanTexture(world);

        uint32_t NX = world.header.demNX;
        uint32_t NZ = world.header.demNZ;
        float minX = world.header.minX, maxX = world.header.maxX;
        float minZ = world.header.minZ, maxZ = world.header.maxZ;
        float spanX = maxX - minX, spanZ = maxZ - minZ;

        std::vector<float> urbanField(NX * NZ, 0.0f);
        for (const auto& b : world.buildings) {
            if (b.footprintXZ.empty()) continue;
            int gx = std::clamp((int)(((b.footprintXZ[0].x - minX) / spanX) * (NX - 1)), 0, (int)NX - 1);
            int gz = std::clamp((int)(((b.footprintXZ[0].z - minZ) / spanZ) * (NZ - 1)), 0, (int)NZ - 1);
            urbanField[gz * NX + gx] = std::min(1.0f, urbanField[gz * NX + gx] + 0.65f);
        }
        smoothGridField(urbanField, NX, NZ, 3);

        std::vector<float> waterField(NX * NZ, 0.0f);
        for (uint32_t i = 0; i < NX * NZ; ++i) {
            waterField[i] = (world.demWaterMask[i] != 0) ? 1.0f : 0.0f;
        }

        std::vector<float> smoothH = world.demHeights;
        smoothGridField(smoothH, NX, NZ, 1);
        for (uint32_t i = 0; i < NX * NZ; ++i) {
            if (world.demWaterMask[i] != 0) smoothH[i] = 0.0f;
            else smoothH[i] = std::max(2.8f, smoothH[i]);
            world.demHeights[i] = smoothH[i];
        }

        std::vector<TerrainVertex> vertices(NX * NZ);
        float stepX = spanX / (NX - 1);
        float stepZ = spanZ / (NZ - 1);

        for (uint32_t gz = 0; gz < NZ; ++gz) {
            float wz = minZ + gz * stepZ;
            for (uint32_t gx = 0; gx < NX; ++gx) {
                float wx = minX + gx * stepX;
                uint32_t idx = gz * NX + gx;

                uint32_t xL = (gx > 0) ? gx - 1 : 0;
                uint32_t xR = (gx + 1 < NX) ? gx + 1 : NX - 1;
                uint32_t zD = (gz > 0) ? gz - 1 : 0;
                uint32_t zU = (gz + 1 < NZ) ? gz + 1 : NZ - 1;

                Vec3f norm = normalize3(Vec3f(smoothH[gz * NX + xL] - smoothH[gz * NX + xR],
                                              stepX + stepZ,
                                              smoothH[zD * NX + gx] - smoothH[zU * NX + gx]));

                TerrainVertex tv;
                tv.x = wx; tv.y = smoothH[idx]; tv.z = wz;
                tv.nx = norm.x; tv.ny = norm.y; tv.nz = norm.z;
                tv.r = (uint8_t)std::clamp((int)(waterField[idx] * 255.0f), 0, 255);
                tv.g = (uint8_t)std::clamp((int)(urbanField[idx] * 255.0f), 0, 255);
                tv.b = 128;
                tv.a = (uint8_t)std::clamp((int)((tv.y / 1000.0f) * 255.0f), 0, 255);
                vertices[idx] = tv;
            }
        }

        glGenBuffers(1, &m_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(TerrainVertex), vertices.data(), GL_STATIC_DRAW);

        // Araziyi 8x12 = 96 Sektor EBO'suna Bol (Kameranin arkasindaki daglari cizmemek icin!)
        uint32_t cellsX = (NX - 1) / SEC_X;
        uint32_t cellsZ = (NZ - 1) / SEC_Z;
        float secSpanX = spanX / SEC_X;
        float secSpanZ = spanZ / SEC_Z;

        for (int sz = 0; sz < SEC_Z; ++sz) {
            for (int sx = 0; sx < SEC_X; ++sx) {
                uint32_t gz0 = sz * cellsZ;
                uint32_t gz1 = (sz + 1 == SEC_Z) ? (NZ - 1) : (gz0 + cellsZ);
                uint32_t gx0 = sx * cellsX;
                uint32_t gx1 = (sx + 1 == SEC_X) ? (NX - 1) : (gx0 + cellsX);

                std::vector<uint32_t> secIdx;
                secIdx.reserve((gx1 - gx0) * (gz1 - gz0) * 6);
                for (uint32_t gz = gz0; gz < gz1; ++gz) {
                    for (uint32_t gx = gx0; gx < gx1; ++gx) {
                        uint32_t i00 = gz * NX + gx;
                        uint32_t i10 = i00 + 1;
                        uint32_t i01 = (gz + 1) * NX + gx;
                        uint32_t i11 = i01 + 1;
                        secIdx.push_back(i00); secIdx.push_back(i01); secIdx.push_back(i10);
                        secIdx.push_back(i10); secIdx.push_back(i01); secIdx.push_back(i11);
                    }
                }

                auto& sec = m_sectors[sz][sx];
                sec.centerX = minX + (sx + 0.5f) * secSpanX;
                sec.centerZ = minZ + (sz + 0.5f) * secSpanZ;
                sec.radius  = std::hypot(secSpanX, secSpanZ) * 0.65f;
                sec.indexCount = (uint32_t)secIdx.size();

                glGenBuffers(1, &sec.ebo);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sec.ebo);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, secIdx.size() * sizeof(uint32_t), secIdx.data(), GL_STATIC_DRAW);
            }
        }

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        return true;
    }

    void render(float simTime, bool nightMode, const Vec3f& sunDir, const Vec3f& camPos, const Vec3f& camFwd, const Vec3f& fogCol) {
        m_shader.use();
        m_shader.setInt("uNoiseTex", 0);
        m_shader.setInt("uRoadMapTex", 1);
        m_shader.setFloat("uTime", simTime);
        m_shader.setFloat("uNightMode", nightMode ? 1.0f : 0.0f);
        m_shader.setVec3("uSunDir", sunDir);
        m_shader.setVec3("uCameraPos", camPos);
        m_shader.setVec3("uFogColor", fogCol);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_noiseTex);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, m_roadMapTex);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        glVertexPointer(3, GL_FLOAT, sizeof(TerrainVertex), reinterpret_cast<const void*>(offsetof(TerrainVertex, x)));
        glNormalPointer(GL_FLOAT, sizeof(TerrainVertex), reinterpret_cast<const void*>(offsetof(TerrainVertex, nx)));
        glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(TerrainVertex), reinterpret_cast<const void*>(offsetof(TerrainVertex, r)));

        float fwdX = camFwd.x, fwdZ = camFwd.z;
        float fwdLen = std::hypot(fwdX, fwdZ);
        if (fwdLen > 1e-4f) { fwdX /= fwdLen; fwdZ /= fwdLen; }

        for (int sz = 0; sz < SEC_Z; ++sz) {
            for (int sx = 0; sx < SEC_X; ++sx) {
                const auto& sec = m_sectors[sz][sx];
                if (!sec.ebo || sec.indexCount == 0) continue;
                float dx = sec.centerX - camPos.x;
                float dz = sec.centerZ - camPos.z;
                float dist = std::hypot(dx, dz);
                if (dist > 19500.0f + sec.radius) continue;
                if (dist > sec.radius * 1.5f) {
                    if ((dx * fwdX + dz * fwdZ) / dist < -0.35f) continue;
                }
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sec.ebo);
                glDrawElements(GL_TRIANGLES, sec.indexCount, GL_UNSIGNED_INT, nullptr);
            }
        }

        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE0);
        ShaderProgram::unbind();
    }
};

} // namespace izmir
