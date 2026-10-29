#pragma once
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include "../core/IzmbFormat.hpp"
#include "../core/FlightInput.hpp"
#include "DynamicWorldRenderer.hpp"

namespace izmir {

struct RadarLineSeg {
    float x0, z0, x1, z1;
    uint8_t cat; // 0: Motorway, 1: Primary, 4: IzbanRailway
};

class HudAndRadarRenderer {
private:
    static const uint8_t* getGlyph(char c) {
        static const uint8_t DIGITS[10][7] = {
            {14,17,19,21,25,17,14}, {4,12,4,4,4,4,14}, {14,17,1,6,8,16,31},
            {31,2,4,2,1,17,14}, {2,6,10,18,31,2,2}, {31,16,30,1,1,17,14},
            {6,8,16,30,17,17,14}, {31,1,2,4,8,8,8}, {14,17,17,14,17,17,14}, {14,17,17,15,1,2,12}
        };
        static const uint8_t LETTERS[26][7] = {
            {14,17,17,31,17,17,17}, {30,17,17,30,17,17,30}, {14,17,16,16,16,17,14},
            {28,18,17,17,17,18,28}, {31,16,16,30,16,16,31}, {31,16,16,30,16,16,16},
            {14,17,16,23,17,17,15}, {17,17,17,31,17,17,17}, {14,4,4,4,4,4,14},
            {7,2,2,2,2,18,12}, {17,18,20,24,20,18,17}, {16,16,16,16,16,16,31},
            {17,27,21,21,17,17,17}, {17,25,21,19,17,17,17}, {14,17,17,17,17,17,14},
            {30,17,17,30,16,16,16}, {14,17,17,17,21,18,13}, {30,17,17,30,20,18,17},
            {15,16,16,14,1,1,30}, {31,4,4,4,4,4,4}, {17,17,17,17,17,17,14},
            {17,17,17,17,17,10,4}, {17,17,17,21,21,27,17}, {17,17,10,4,10,17,17},
            {17,17,10,4,4,4,4}, {31,1,2,4,8,16,31}
        };
        static const uint8_t SYM_SPC[7] = {0,0,0,0,0,0,0};
        static const uint8_t SYM_DOT[7] = {0,0,0,0,0,12,12};
        static const uint8_t SYM_COL[7] = {0,12,12,0,12,12,0};
        static const uint8_t SYM_DSH[7] = {0,0,0,31,0,0,0};
        static const uint8_t SYM_SLS[7] = {1,2,2,4,8,8,16};
        static const uint8_t SYM_PCT[7] = {25,26,4,8,16,19,19};
        static const uint8_t SYM_LBR[7] = {14,8,8,8,8,8,14};
        static const uint8_t SYM_RBR[7] = {14,2,2,2,2,2,14};

        if (c >= '0' && c <= '9') return DIGITS[c - '0'];
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c >= 'A' && c <= 'Z') return LETTERS[c - 'A'];
        if (c == '.') return SYM_DOT;
        if (c == ':') return SYM_COL;
        if (c == '-') return SYM_DSH;
        if (c == '/') return SYM_SLS;
        if (c == '%') return SYM_PCT;
        if (c == '[' || c == '(') return SYM_LBR;
        if (c == ']' || c == ')') return SYM_RBR;
        return SYM_SPC;
    }

public:
    static void drawText(float x, float y, float scale, const std::string& text, float r, float g, float b) {
        glColor3f(r, g, b);
        glBegin(GL_QUADS);
        float curX = x;
        for (char c : text) {
            const uint8_t* rows = getGlyph(c);
            for (int ry = 0; ry < 7; ++ry) {
                uint8_t bits = rows[ry];
                for (int rx = 0; rx < 5; ++rx) {
                    if (bits & (1 << (4 - rx))) {
                        float px = curX + rx * scale, py = y + ry * scale;
                        glVertex2f(px, py); glVertex2f(px + scale, py);
                        glVertex2f(px + scale, py + scale); glVertex2f(px, py + scale);
                    }
                }
            }
            curX += 6.0f * scale;
        }
        glEnd();
    }

    static void drawRect(float x, float y, float w, float h, float r, float g, float b, float a = 1.0f) {
        glColor4f(r, g, b, a);
        glBegin(GL_QUADS);
        glVertex2f(x, y); glVertex2f(x + w, y); glVertex2f(x + w, y + h); glVertex2f(x, y + h);
        glEnd();
    }

    static void renderHudAndVectorRadar(int winW, int winH,
                                        const IzmbWorldData& world,
                                        const DynamicWorldRenderer& dynWorld,
                                        const FlightInput& input,
                                        const Vec3f& pos, float yawRad, float pitchRad,
                                        float speedKmh, float fps,
                                        const std::string& zoneName,
                                        double curLat, double curLon) {
        // ILK KAREDE RADAR HATLARINI SADECE 1 KEZ ONBELLEGE AL (80,000 yolu her karede tarama darbogazini %100 yok eder!)
        static bool s_radarCacheBuilt = false;
        static std::vector<RadarLineSeg> s_radarLines;
        if (!s_radarCacheBuilt) {
            s_radarCacheBuilt = true;
            s_radarLines.reserve(12000);
            for (const auto& w : world.ways) {
                if (w.category != WayCategory::IzbanRailway && w.category != WayCategory::Motorway && w.category != WayCategory::Primary) continue;
                uint8_t cat = (uint8_t)w.category;
                size_t step = (w.centerline.size() > 8) ? 2 : 1;
                for (size_t i = 0; i + step < w.centerline.size(); i += step) {
                    s_radarLines.push_back({w.centerline[i].x, w.centerline[i].z,
                                            w.centerline[i + step].x, w.centerline[i + step].z, cat});
                }
            }
        }

        glUseProgram(0);
        glDisable(GL_LIGHTING);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, winW, winH, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // 1. Ust Bilgi Paneli
        float topW = 750.0f, topX = (winW - topW) * 0.5f;
        drawRect(topX, 12.0f, topW, 56.0f, 0.04f, 0.07f, 0.13f, 0.88f);
        drawText(topX + 16.0f, 20.0f, 1.85f, "IZMIR 3D SIMULATOR v12.0 TURBO - BOLGE: " + zoneName, 0.25f, 0.94f, 1.0f);
        std::string modeStr = input.freeCamMode
            ? "MOD: SERBEST DRONE [F: UCAK] | [G]: GUN BATIMI | [T]: GECE | [WASD/OKLAR]: HAREKET"
            : "MOD: 64-BIT UCAK + JOYSTICK [F: DRONE] | [G]: GUN BATIMI | [T]: GECE | [N/P]: BOLGE";
        drawText(topX + 16.0f, 44.0f, 1.42f, modeStr, 1.0f, 0.90f, 0.58f);

        // 2. Sol Ust Telemetri Paneli
        drawRect(16.0f, 12.0f, 415.0f, 138.0f, 0.04f, 0.07f, 0.13f, 0.88f);
        bool wat = false;
        int aglM = std::max(0, (int)(pos.y - world.sampleHeightAt(pos.x, pos.z, wat)));
        char l1[80], l2[80], l3[80], l4[80], l5[80];
        std::snprintf(l1, sizeof(l1), "IRTIFA   : %4d M (AGL: %4d M)  FPS: %d", (int)pos.y, aglM, (int)fps);
        std::snprintf(l2, sizeof(l2), "HIZ      : %4d KMH (%3d KTS)   GAZ: %3d%%", (int)speedKmh, (int)(speedKmh / 1.852f), (int)(input.throttle * 100.0f));
        std::snprintf(l3, sizeof(l3), "IZMIR GPS: %.4f N   %.4f E", std::fabs(curLat), std::fabs(curLon));
        std::snprintf(l4, sizeof(l4), "KONTROLCU: %s", input.deviceName.c_str());
        std::snprintf(l5, sizeof(l5), "EKSENLER : ROLL: %.2f  PITCH: %.2f  YAW: %.2f", input.roll, input.pitch, input.yaw);

        drawText(26.0f, 22.0f, 1.72f, l1, 0.95f, 0.98f, 1.0f);
        drawText(26.0f, 45.0f, 1.72f, l2, 0.45f, 0.98f, 0.68f);
        drawText(26.0f, 68.0f, 1.72f, l3, 0.35f, 0.92f, 1.0f);
        drawText(26.0f, 92.0f, 1.65f, l4, (input.jsFd >= 0) ? 0.30f : 1.0f, 0.95f, 0.45f);
        drawText(26.0f, 115.0f, 1.55f, l5, 0.88f, 0.88f, 0.94f);

        // 3. Sag Alt: Hizli Onbellekli Vektor Radar
        float mapSize = 230.0f;
        float mapX = winW - mapSize - 18.0f;
        float mapY = winH - mapSize - 58.0f;
        drawRect(mapX - 4.0f, mapY - 24.0f, mapSize + 8.0f, mapSize + 30.0f, 0.03f, 0.06f, 0.11f, 0.94f);
        drawText(mapX + 6.0f, mapY - 18.0f, 1.45f, "IZMIR VEKTOR GPS & IZBAN RADAR [Z]", 0.35f, 0.94f, 1.0f);

        float radarRangeM = input.radarWide ? 14000.0f : 5500.0f;
        auto worldToRadar = [&](float wx, float wz, float& outPx, float& outPy) -> bool {
            float relX = (wx - pos.x) / radarRangeM;
            float relZ = (wz - pos.z) / radarRangeM;
            if (relX < -0.98f || relX > 0.98f || relZ < -0.98f || relZ > 0.98f) return false;
            outPx = mapX + (0.5f + relX * 0.5f) * mapSize;
            outPy = mapY + (0.5f + relZ * 0.5f) * mapSize;
            return true;
        };

        drawRect(mapX, mapY, mapSize, mapSize, 0.12f, 0.18f, 0.14f, 1.0f);
        glBegin(GL_QUADS);
        glColor3f(0.06f, 0.28f, 0.48f);
        const int RN = 28;
        float cellPx = mapSize / RN;
        uint32_t NX = world.header.demNX, NZ = world.header.demNZ;
        float minX = world.header.minX, spanX = world.header.maxX - world.header.minX;
        float minZ = world.header.minZ, spanZ = world.header.maxZ - world.header.minZ;
        for (int rz = 0; rz < RN; ++rz) {
            float wz = pos.z + ((rz + 0.5f) / RN * 2.0f - 1.0f) * radarRangeM;
            int gz = std::clamp((int)(((wz - minZ) / spanZ) * (NZ - 1)), 0, (int)NZ - 1);
            for (int rx = 0; rx < RN; ++rx) {
                float wx = pos.x + ((rx + 0.5f) / RN * 2.0f - 1.0f) * radarRangeM;
                int gx = std::clamp((int)(((wx - minX) / spanX) * (NX - 1)), 0, (int)NX - 1);
                if (world.demWaterMask[gz * NX + gx] != 0) {
                    float px = mapX + rx * cellPx, py = mapY + rz * cellPx;
                    glVertex2f(px, py); glVertex2f(px + cellPx, py);
                    glVertex2f(px + cellPx, py + cellPx); glVertex2f(px, py + cellPx);
                }
            }
        }
        glEnd();

        glLineWidth(2.0f);
        glBegin(GL_LINES);
        for (const auto& seg : s_radarLines) {
            float x0, y0, x1, y1;
            if (worldToRadar(seg.x0, seg.z0, x0, y0) && worldToRadar(seg.x1, seg.z1, x1, y1)) {
                if (seg.cat == 4)      glColor3f(0.15f, 0.90f, 1.0f);
                else if (seg.cat == 0) glColor3f(0.85f, 0.68f, 0.25f);
                else                   glColor3f(0.45f, 0.48f, 0.52f);
                glVertex2f(x0, y0); glVertex2f(x1, y1);
            }
        }
        glEnd();
        glLineWidth(1.0f);

        for (const auto& tr : dynWorld.currentTrainPositions) {
            float tx, ty;
            if (worldToRadar(tr.x, tr.z, tx, ty)) {
                drawRect(tx - 2.5f, ty - 2.5f, 5.0f, 5.0f, 1.0f, 0.92f, 0.15f, 1.0f);
            }
        }

        for (const auto& st : world.stations) {
            float sx, sy;
            if (worldToRadar(st.x, st.z, sx, sy)) {
                drawRect(sx - 3.0f, sy - 3.0f, 6.0f, 6.0f, 0.15f, 0.75f, 1.0f, 1.0f);
                drawText(sx + 5.0f, sy - 4.0f, 1.22f, st.code, 0.88f, 0.98f, 1.0f);
            }
        }

        float cx = mapX + mapSize * 0.5f, cy = mapY + mapSize * 0.5f;
        float sY = std::sin(yawRad), cY = std::cos(yawRad);
        glColor3f(1.0f, 0.20f, 0.20f);
        glBegin(GL_TRIANGLES);
        glVertex2f(cx + sY * 10.0f, cy + cY * 10.0f);
        glVertex2f(cx - cY * 6.5f - sY * 6.0f, cy + sY * 6.5f - cY * 6.0f);
        glVertex2f(cx + cY * 6.5f - sY * 6.0f, cy - sY * 6.5f - cY * 6.0f);
        glEnd();

        drawRect(16.0f, winH - 42.0f, winW - 32.0f, 30.0f, 0.04f, 0.07f, 0.13f, 0.88f);
        drawText(24.0f, winH - 32.0f, 1.38f,
                 "[1]:KONAK [2]:ALSANCAK [3]:BAYRAKLI [4]:KARSIYAKA [5]:EGEKENT2 [6]:ULUKENT [7]:MENEMEN [8]:CIGLI [9]:MAVISEHIR [0]:LTBJ",
                 0.86f, 0.94f, 1.0f);

        glEnable(GL_DEPTH_TEST);
    }
};

} // namespace izmir
