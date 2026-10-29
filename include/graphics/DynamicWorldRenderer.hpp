#pragma once
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include "../core/GeoProjection.hpp"
#include "../core/IzmbFormat.hpp"
#include "../core/Mat4.hpp"

namespace izmir {

class DynamicWorldRenderer {
public:
    struct ActiveTrainPos {
        float x, z;
    };
    std::vector<ActiveTrainPos> currentTrainPositions;

private:
    struct TrackNode {
        float x, z;
        float cumDist;
        float yawDeg;
    };
    std::vector<TrackNode> m_masterIzbanRoute;
    std::vector<const WayRibbonRecord*> m_highwayPool;
    bool m_routeBuilt = false;

    void buildRoutes(const IzmbWorldData& world) {
        m_routeBuilt = true;

        // 1. Otoyol ve Ana Bulvar Trafik Havuzu (Canli Arac Akisi Icin)
        for (const auto& w : world.ways) {
            if ((w.category == WayCategory::Motorway || w.category == WayCategory::Primary) && w.centerline.size() >= 4) {
                float len = std::hypot(w.centerline.back().x - w.centerline.front().x,
                                       w.centerline.back().z - w.centerline.front().z);
                if (len > 120.0f) m_highwayPool.push_back(&w);
            }
        }

        // 2. Kesintisiz IZBAN Demiryolu Omurgasi
        if (world.stations.size() < 2) return;
        std::vector<Vec2d> spineWaypoints;
        for (const auto& st : world.stations) {
            std::string nm(st.name);
            if (nm == "ALSANCAK" || nm == "BASMANE" || nm == "KEMER") continue;
            spineWaypoints.push_back({st.x, st.z});
        }
        std::sort(spineWaypoints.begin(), spineWaypoints.end(), [](const Vec2d& a, const Vec2d& b) {
            return a.z < b.z;
        });

        std::vector<Vec2d> rawPts;
        for (size_t i = 0; i + 1 < spineWaypoints.size(); ++i) {
            Vec2d pA = spineWaypoints[i];
            Vec2d pB = spineWaypoints[i + 1];
            double segLen = std::hypot(pB.x - pA.x, pB.z - pA.z);
            int steps = std::max(1, (int)(segLen / 20.0));
            for (int s = 0; s < steps; ++s) {
                double t = (double)s / steps;
                float qx = (float)(pA.x + (pB.x - pA.x) * t);
                float qz = (float)(pA.z + (pB.z - pA.z) * t);

                float bestDSq = 180.0f * 180.0f;
                float snapX = qx, snapZ = qz;
                for (const auto& w : world.ways) {
                    if (w.category != WayCategory::IzbanRailway) continue;
                    for (size_t k = 0; k + 1 < w.centerline.size(); ++k) {
                        const Vec3f& r0 = w.centerline[k];
                        const Vec3f& r1 = w.centerline[k + 1];
                        float dx = r1.x - r0.x, dz = r1.z - r0.z;
                        float lSq = dx * dx + dz * dz;
                        if (lSq < 0.5f) continue;
                        float u = std::clamp(((qx - r0.x) * dx + (qz - r0.z) * dz) / lSq, 0.0f, 1.0f);
                        float px = r0.x + u * dx, pz = r0.z + u * dz;
                        float dSq = (qx - px) * (qx - px) + (qz - pz) * (qz - pz);
                        if (dSq < bestDSq) {
                            bestDSq = dSq;
                            snapX = px;
                            snapZ = pz;
                        }
                    }
                }
                rawPts.push_back({snapX, snapZ});
            }
        }

        for (int pass = 0; pass < 2; ++pass) {
            std::vector<Vec2d> sm = rawPts;
            for (size_t i = 1; i + 1 < rawPts.size(); ++i) {
                sm[i].x = rawPts[i - 1].x * 0.25 + rawPts[i].x * 0.50 + rawPts[i + 1].x * 0.25;
                sm[i].z = rawPts[i - 1].z * 0.25 + rawPts[i].z * 0.50 + rawPts[i + 1].z * 0.25;
            }
            rawPts.swap(sm);
        }

        float totalD = 0.0f;
        for (size_t i = 0; i < rawPts.size(); ++i) {
            if (i > 0) {
                totalD += (float)std::hypot(rawPts[i].x - rawPts[i - 1].x, rawPts[i].z - rawPts[i - 1].z);
            }
            size_t nxt = std::min(i + 1, rawPts.size() - 1);
            size_t prv = (i > 0) ? (i - 1) : 0;
            float yawDeg = (float)std::atan2(rawPts[nxt].x - rawPts[prv].x, rawPts[nxt].z - rawPts[prv].z) * 57.29578f;
            m_masterIzbanRoute.push_back({(float)rawPts[i].x, (float)rawPts[i].z, totalD, yawDeg});
        }
    }

public:
    static void drawBox(float x0, float y0, float z0, float x1, float y1, float z1) {
        glBegin(GL_QUADS);
        glNormal3f(0, 1, 0);  glVertex3f(x0, y1, z0); glVertex3f(x0, y1, z1); glVertex3f(x1, y1, z1); glVertex3f(x1, y1, z0);
        glNormal3f(0, -1, 0); glVertex3f(x0, y0, z0); glVertex3f(x1, y0, z0); glVertex3f(x1, y0, z1); glVertex3f(x0, y0, z1);
        glNormal3f(0, 0, -1); glVertex3f(x0, y0, z0); glVertex3f(x0, y1, z0); glVertex3f(x1, y1, z0); glVertex3f(x1, y0, z0);
        glNormal3f(0, 0, 1);  glVertex3f(x0, y0, z1); glVertex3f(x1, y0, z1); glVertex3f(x1, y1, z1); glVertex3f(x0, y1, z1);
        glNormal3f(-1, 0, 0); glVertex3f(x0, y0, z0); glVertex3f(x0, y0, z1); glVertex3f(x0, y1, z1); glVertex3f(x0, y1, z0);
        glNormal3f(1, 0, 0);  glVertex3f(x1, y0, z0); glVertex3f(x1, y1, z0); glVertex3f(x1, y1, z1); glVertex3f(x1, y0, z1);
        glEnd();
    }

    static void drawCylinderZ(float x, float y, float z0, float z1, float r0, float r1, int segs) {
        glBegin(GL_QUAD_STRIP);
        for (int i = 0; i <= segs; ++i) {
            float a = i * 2.0f * (float)GeoProjection::PI / segs;
            float nx = std::cos(a), ny = std::sin(a);
            glNormal3f(nx, ny, 0.0f);
            glVertex3f(x + nx * r0, y + ny * r0, z0);
            glVertex3f(x + nx * r1, y + ny * r1, z1);
        }
        glEnd();
    }

    static void drawIzbanStationStarBeacons(const IzmbWorldData& world, float simTime) {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        float bobY = std::sin(simTime * 2.6f) * 3.0f;
        float rotDeg = simTime * 75.0f;

        for (const auto& st : world.stations) {
            bool wat = false;
            float gy = world.sampleHeightAt(st.x, st.z, wat);
            float starY = gy + 48.0f + bobY;

            glDepthMask(GL_FALSE);
            glBegin(GL_QUADS);
            glColor4f(0.15f, 0.85f, 1.0f, 0.48f);
            glVertex3f(st.x - 1.8f, gy + 10.0f, st.z);
            glVertex3f(st.x + 1.8f, gy + 10.0f, st.z);
            glColor4f(1.0f, 0.90f, 0.25f, 0.05f);
            glVertex3f(st.x + 5.5f, starY + 25.0f, st.z);
            glVertex3f(st.x - 5.5f, starY + 25.0f, st.z);

            glColor4f(0.15f, 0.85f, 1.0f, 0.48f);
            glVertex3f(st.x, gy + 10.0f, st.z - 1.8f);
            glVertex3f(st.x, gy + 10.0f, st.z + 1.8f);
            glColor4f(1.0f, 0.90f, 0.25f, 0.05f);
            glVertex3f(st.x, starY + 25.0f, st.z + 5.5f);
            glVertex3f(st.x, starY + 25.0f, st.z - 5.5f);
            glEnd();
            glDepthMask(GL_TRUE);

            glPushMatrix();
            glTranslatef(st.x, starY, st.z);
            glRotatef(rotDeg, 0.0f, 1.0f, 0.0f);

            const float R_OUT = 13.5f;
            const float R_IN  = 5.4f;
            const float THICK = 3.4f;

            glBegin(GL_TRIANGLES);
            for (int i = 0; i < 10; ++i) {
                float a0 = (i * 36.0f - 90.0f) * (float)GeoProjection::DEG2RAD;
                float a1 = ((i + 1) * 36.0f - 90.0f) * (float)GeoProjection::DEG2RAD;
                float r0 = (i % 2 == 0) ? R_OUT : R_IN;
                float r1 = ((i + 1) % 2 == 0) ? R_OUT : R_IN;

                float x0 = std::cos(a0) * r0, y0 = -std::sin(a0) * r0;
                float x1 = std::cos(a1) * r1, y1 = -std::sin(a1) * r1;

                glColor3f(1.0f, 0.98f, 0.82f);
                glVertex3f(0.0f, 0.0f, THICK);
                glColor3f(1.0f, 0.82f, 0.12f);
                glVertex3f(x0, y0, 0.0f);
                glColor3f(0.96f, 0.62f, 0.08f);
                glVertex3f(x1, y1, 0.0f);

                glColor3f(1.0f, 0.98f, 0.82f);
                glVertex3f(0.0f, 0.0f, -THICK);
                glColor3f(0.96f, 0.62f, 0.08f);
                glVertex3f(x1, y1, 0.0f);
                glColor3f(1.0f, 0.82f, 0.12f);
                glVertex3f(x0, y0, 0.0f);
            }
            glEnd();
            glPopMatrix();
        }
        glEnable(GL_LIGHTING);
    }

    static void drawAirportRunways(const IzmbWorldData& world, const GeoProjection& proj) {
        struct RunwayDef { double lat, lon; float yawDeg, lengthM, widthM; };
        static const RunwayDef RUNWAYS[2] = {
            { 38.2924, 27.1565, 345.0f, 3200.0f, 48.0f },
            { 38.5065, 27.0125, 350.0f, 2800.0f, 45.0f }
        };

        for (const auto& rw : RUNWAYS) {
            Vec2d xz = proj.latLonToWorldXZ(rw.lat, rw.lon);
            bool wat = false;
            float gy = world.sampleHeightAt((float)xz.x, (float)xz.z, wat) + 0.55f;
            float yaw = (180.0f - rw.yawDeg) * (float)GeoProjection::DEG2RAD;

            glPushMatrix();
            glTranslatef((float)xz.x, gy, (float)xz.z);
            glRotatef(yaw * 57.29578f, 0, 1, 0);

            glColor3f(0.20f, 0.22f, 0.25f);
            drawBox(-rw.widthM * 0.5f, 0.0f, -rw.lengthM * 0.5f, rw.widthM * 0.5f, 0.22f, rw.lengthM * 0.5f);

            glColor3f(0.94f, 0.95f, 0.97f);
            for (float z = -rw.lengthM * 0.46f; z < rw.lengthM * 0.46f; z += 65.0f) {
                drawBox(-0.75f, 0.24f, z, 0.75f, 0.28f, z + 34.0f);
            }
            for (float x = -rw.widthM * 0.38f; x <= rw.widthM * 0.38f; x += 4.2f) {
                drawBox(x - 1.0f, 0.24f, -rw.lengthM * 0.48f, x + 1.0f, 0.28f, -rw.lengthM * 0.48f + 42.0f);
                drawBox(x - 1.0f, 0.24f,  rw.lengthM * 0.48f - 42.0f, x + 1.0f, 0.28f, rw.lengthM * 0.48f);
            }
            glPopMatrix();
        }
    }

    void renderDynamicVehicles(const IzmbWorldData& world, const GeoProjection& proj, float simTime, bool nightMode, const Vec3f& camPos) {
        if (!m_routeBuilt) {
            buildRoutes(world);
        }

        glUseProgram(0);
        glDisable(GL_TEXTURE_2D);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);

        drawAirportRunways(world, proj);

        // ====================================================================
        // 1. CANLI OTOYOL VE BULVAR ARAC TRAFIGI (OTOMOBIL, ESHOT OTOBUS, TIR)
        // ====================================================================
        if (!m_highwayPool.empty()) {
            int maxWays = std::min(85, (int)m_highwayPool.size());
            for (int wIdx = 0; wIdx < maxWays; ++wIdx) {
                const auto* w = m_highwayPool[(wIdx * 5) % m_highwayPool.size()];
                const Vec3f& mid = w->centerline[w->centerline.size() / 2];
                if (std::hypot(mid.x - camPos.x, mid.z - camPos.z) > 5500.0f) continue;

                int nPts = (int)w->centerline.size();
                for (int c = 0; c < 2; ++c) {
                    float tNorm = std::fmod(simTime * (0.16f + (wIdx % 5) * 0.03f) + c * 0.48f + wIdx * 0.13f, 1.0f);
                    float fIdx = tNorm * (nPts - 1);
                    int i0 = std::clamp((int)fIdx, 0, nPts - 2);
                    float frac = fIdx - i0;

                    const Vec3f& p0 = w->centerline[i0];
                    const Vec3f& p1 = w->centerline[i0 + 1];
                    float dx = p1.x - p0.x, dz = p1.z - p0.z;
                    float segL = std::hypot(dx, dz);
                    if (segL < 0.5f) continue;

                    float vx = p0.x + dx * frac + (-dz / segL) * 1.6f;
                    float vz = p0.z + dz * frac + ( dx / segL) * 1.6f;
                    bool wat = false;
                    float vy = world.sampleHeightAt(vx, vz, wat) + (w->isBridge ? 4.8f : 0.45f);
                    float yawDeg = std::atan2(dx, dz) * 57.29578f;

                    glPushMatrix();
                    glTranslatef(vx, vy, vz);
                    glRotatef(yawDeg, 0, 1, 0);

                    bool isBus = ((wIdx + c) % 7 == 0); // Kirmizi-Beyaz Izmir ESHOT Otobusu!
                    if (isBus) {
                        glColor3f(0.88f, 0.16f, 0.16f);
                        drawBox(-1.25f, 0.2f, -5.8f, 1.25f, 2.8f, 5.8f);
                        glColor3f(0.95f, 0.96f, 0.98f);
                        drawBox(-1.27f, 2.0f, -5.6f, 1.27f, 2.85f, 5.6f);
                    } else {
                        int colPick = (wIdx + c) % 4;
                        if (colPick == 0)      glColor3f(0.92f, 0.94f, 0.96f);
                        else if (colPick == 1) glColor3f(0.22f, 0.25f, 0.30f);
                        else if (colPick == 2) glColor3f(0.78f, 0.18f, 0.16f);
                        else                   glColor3f(0.18f, 0.42f, 0.72f);
                        drawBox(-0.95f, 0.15f, -2.2f, 0.95f, 1.35f, 2.2f);
                    }

                    // Gece Modunda Parlak On Farlar ve Kirmizi Arka Stop Lambalari
                    if (nightMode) {
                        glDisable(GL_LIGHTING);
                        float fZ = isBus ? 5.9f : 2.3f;
                        glColor3f(1.0f, 0.96f, 0.82f);
                        drawBox(-0.85f, 0.4f, fZ, -0.45f, 0.75f, fZ + 0.3f);
                        drawBox( 0.45f, 0.4f, fZ,  0.85f, 0.75f, fZ + 0.3f);
                        glColor3f(1.0f, 0.12f, 0.12f);
                        drawBox(-0.85f, 0.4f, -fZ - 0.3f, -0.45f, 0.75f, -fZ);
                        drawBox( 0.45f, 0.4f, -fZ - 0.3f,  0.85f, 0.75f, -fZ);
                        glEnable(GL_LIGHTING);
                    }
                    glPopMatrix();
                }
            }
        }

        // ====================================================================
        // 2. KESINTISIZ 55 KM HAT BOYUNCA GIDEN 10 ADET 3D IZBAN TRENI
        // ====================================================================
        currentTrainPositions.clear();
        if (m_masterIzbanRoute.size() >= 4) {
            float totalLen = m_masterIzbanRoute.back().cumDist;
            if (totalLen > 100.0f) {
                for (int tr = 0; tr < 10; ++tr) {
                    float travelM = std::fmod(simTime * 24.0f + tr * (totalLen / 5.0f), totalLen * 2.0f);
                    bool reverseDir = (travelM > totalLen);
                    float targetDist = reverseDir ? (totalLen * 2.0f - travelM) : travelM;

                    auto it = std::lower_bound(
                        m_masterIzbanRoute.begin(), m_masterIzbanRoute.end(), targetDist,
                        [](const TrackNode& nd, float d) { return nd.cumDist < d; });

                    size_t idx1 = std::clamp((size_t)std::distance(m_masterIzbanRoute.begin(), it),
                                             (size_t)1, m_masterIzbanRoute.size() - 1);
                    size_t idx0 = idx1 - 1;
                    const auto& n0 = m_masterIzbanRoute[idx0];
                    const auto& n1 = m_masterIzbanRoute[idx1];
                    float span = std::max(0.1f, n1.cumDist - n0.cumDist);
                    float f = std::clamp((targetDist - n0.cumDist) / span, 0.0f, 1.0f);

                    float tx = n0.x + (n1.x - n0.x) * f;
                    float tz = n0.z + (n1.z - n0.z) * f;
                    bool wat = false;
                    float ty = world.sampleHeightAt(tx, tz, wat) + 0.65f;
                    float yawDeg = n0.yawDeg + (reverseDir ? 180.0f : 0.0f);

                    currentTrainPositions.push_back({tx, tz});

                    glPushMatrix();
                    glTranslatef(tx, ty, tz);
                    glRotatef(yawDeg, 0, 1, 0);

                    for (int car = -1; car <= 1; ++car) {
                        float cz = car * 18.0f;
                        glColor3f(0.95f, 0.96f, 0.98f);
                        drawBox(-1.45f, 0.4f, cz - 8.2f, 1.45f, 3.6f, cz + 8.2f);
                        glColor3f(0.08f, 0.42f, 0.85f);
                        drawBox(-1.50f, 1.2f, cz - 8.3f, 1.50f, 1.9f, cz + 8.3f);
                        glColor3f(nightMode ? 0.98f : 0.16f, nightMode ? 0.88f : 0.24f, nightMode ? 0.55f : 0.34f);
                        drawBox(-1.52f, 2.0f, cz - 7.5f, 1.52f, 3.0f, cz + 7.5f);
                        glColor3f(0.95f, 0.42f, 0.12f);
                        drawBox(-1.51f, 0.9f, cz - 8.2f, 1.51f, 1.2f, cz + 8.2f);
                    }
                    glPopMatrix();
                }
            }
        }

        // ====================================================================
        // 3. KORFEZDE KOPUK IZLI 4 ADET 3D IZDENIZ VAPURU
        // ====================================================================
        struct FerryRoute { double latA, lonA, latB, lonB; float speed, phase; };
        static const FerryRoute FERRIES[4] = {
            { 38.4198, 27.1245, 38.4535, 27.1185, 0.035f, 0.15f },
            { 38.4395, 27.1395, 38.4535, 27.1185, 0.042f, 0.60f },
            { 38.4198, 27.1245, 38.4535, 27.0965, 0.030f, 0.35f },
            { 38.4025, 27.0825, 38.4535, 27.0965, 0.032f, 0.80f }
        };

        for (int i = 0; i < 4; ++i) {
            Vec2d pA = proj.latLonToWorldXZ(FERRIES[i].latA, FERRIES[i].lonA);
            Vec2d pB = proj.latLonToWorldXZ(FERRIES[i].latB, FERRIES[i].lonB);
            float rawT = std::fmod(simTime * FERRIES[i].speed + FERRIES[i].phase, 2.0f);
            bool ret = (rawT > 1.0f);
            float t = ret ? (2.0f - rawT) : rawT;

            float fx = (float)(pA.x + (pB.x - pA.x) * t);
            float fz = (float)(pA.z + (pB.z - pA.z) * t);
            float dirX = (float)(ret ? (pA.x - pB.x) : (pB.x - pA.x));
            float dirZ = (float)(ret ? (pA.z - pB.z) : (pB.z - pA.z));
            float yawDeg = std::atan2(dirX, dirZ) * 57.29578f;

            glPushMatrix();
            glTranslatef(fx, 0.3f, fz);
            glRotatef(yawDeg, 0, 1, 0);
            // Beyaz Kopuk Izi (Wake)
            glColor3f(0.82f, 0.90f, 0.96f);
            drawBox(-9.5f, -0.1f, -52.0f, 9.5f, 0.05f, -24.0f);
            // Vapur Govdesi
            glColor3f(0.96f, 0.97f, 0.99f);
            drawBox(-7.5f, 0.0f, -26.0f, 7.5f, 4.0f, 26.0f);
            glColor3f(0.10f, 0.45f, 0.82f);
            drawBox(-6.8f, 4.0f, -20.0f, 6.8f, 7.2f, 20.0f);
            glColor3f(0.96f, 0.97f, 0.99f);
            drawBox(-5.0f, 7.2f, -9.0f, 5.0f, 9.8f, 12.0f);
            glColor3f(0.92f, 0.72f, 0.18f);
            drawBox(-1.6f, 9.8f, -3.5f, 1.6f, 13.8f, 1.0f);
            glPopMatrix();
        }

        drawIzbanStationStarBeacons(world, simTime);
    }

    static void renderAircraft3D(int model, float propAngle, float rollIn, float pitchIn) {
        glUseProgram(0);
        glDisable(GL_TEXTURE_2D);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);

        if (model == 0) {
            glColor3f(0.96f, 0.97f, 0.99f);
            drawCylinderZ(0.0f, 0.00f,  0.80f,  1.85f, 0.44f, 0.24f, 14);
            drawCylinderZ(0.0f, 0.02f, -0.75f,  0.80f, 0.48f, 0.44f, 14);
            drawCylinderZ(0.0f, 0.06f, -3.10f, -0.75f, 0.14f, 0.48f, 14);
            glColor3f(0.12f, 0.45f, 0.88f);
            drawBox(-0.49f, -0.04f, -2.50f, 0.49f, 0.08f, 1.40f);
            glColor3f(0.90f, 0.18f, 0.15f);
            drawBox(-0.495f, -0.10f, -2.20f, 0.495f, -0.04f, 1.20f);
            glColor3f(0.14f, 0.24f, 0.38f);
            drawCylinderZ(0.0f, 0.24f, -0.60f, 0.72f, 0.40f, 0.28f, 12);
            glColor3f(0.96f, 0.97f, 0.99f);
            drawBox(-4.80f, 0.56f, -0.20f, 4.80f, 0.65f, 0.68f);
            glColor3f(0.95f, 0.15f, 0.15f);
            drawBox(-4.86f, 0.55f, -0.22f, -4.55f, 0.66f, 0.70f);
            glColor3f(0.12f, 0.90f, 0.30f);
            drawBox( 4.55f, 0.55f, -0.22f,  4.86f, 0.66f, 0.70f);
            drawBox(-1.55f, 0.16f, -3.05f, 1.55f, 0.23f, -2.45f);
            drawBox(-0.04f, 0.15f, -3.10f, 0.04f, 1.35f, -2.25f);
            glPushMatrix();
            glTranslatef(0.0f, 0.0f, 1.90f);
            glRotatef(propAngle * 57.2958f, 0, 0, 1);
            glColor3f(0.16f, 0.18f, 0.22f);
            drawBox(-1.12f, -0.07f, -0.01f, 1.12f, 0.07f, 0.02f);
            glPopMatrix();
        } else if (model == 1) {
            glColor3f(0.92f, 0.94f, 0.96f);
            drawCylinderZ(0.0f, 0.0f, -2.80f, 1.20f, 0.46f, 0.42f, 14);
            drawCylinderZ(0.0f, -0.02f, 1.20f, 2.85f, 0.42f, 0.04f, 14);
            glColor3f(0.15f, 0.26f, 0.40f);
            drawCylinderZ(0.0f, 0.24f, -0.60f, 1.35f, 0.32f, 0.12f, 12);
            glColor3f(0.90f, 0.14f, 0.14f);
            drawBox(-4.40f, -0.04f, -1.20f, 4.40f, 0.05f, 0.45f);
            drawBox(-0.58f, 0.15f, -2.75f, -0.46f, 1.25f, -1.85f);
            drawBox( 0.46f, 0.15f, -2.75f,  0.58f, 1.25f, -1.85f);
        } else {
            glColor3f(0.98f, 0.98f, 1.0f);
            drawCylinderZ(0.0f, 0.0f, -2.90f, 1.90f, 0.16f, 0.28f, 12);
            drawBox(-6.60f, 0.10f, -0.15f, 6.60f, 0.18f, 0.35f);
            drawBox(-0.04f, 0.10f, -2.95f, 0.04f, 1.20f, -2.40f);
            drawBox(-1.25f, 1.15f, -3.00f, 1.25f, 1.22f, -2.50f);
        }
    }
};

} // namespace izmir
