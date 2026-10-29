#define GL_GLEXT_PROTOTYPES
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <GL/gl.h>
#include <GL/glx.h>

#include "core/GeoProjection.hpp"
#include "core/IzmbFormat.hpp"
#include "core/Mat4.hpp"
#include "core/FlightInput.hpp"
#include "graphics/TerrainRenderer.hpp"
#include "graphics/WayAndRailRenderer.hpp"
#include "graphics/BuildingRenderer.hpp"
#include "graphics/DynamicWorldRenderer.hpp"
#include "graphics/HudAndRadarRenderer.hpp"

using namespace izmir;

void setPerspective(double fovYDeg, double aspect, double zNear, double zFar) {
    double fH = std::tan(fovYDeg * GeoProjection::PI / 360.0) * zNear;
    double fW = fH * aspect;
    glFrustum(-fW, fW, -fH, fH, zNear, zFar);
}

void setLookAt(const Vec3f& eye, const Vec3f& center, const Vec3f& up) {
    Vec3f f = normalize3(center - eye);
    Vec3f s = normalize3(cross3(f, up));
    Vec3f u = cross3(s, f);
    float m[16] = {
         s.x,  u.x, -f.x, 0.0f,
         s.y,  u.y, -f.y, 0.0f,
         s.z,  u.z, -f.z, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    glMultMatrixf(m);
    glTranslatef(-eye.x, -eye.y, -eye.z);
}

void computeSkyColors(bool nightMode, bool goldenSunset, Vec3f& outZenith, Vec3f& outHorizon) {
    if (nightMode) {
        outZenith  = Vec3f(0.012f, 0.024f, 0.068f);
        outHorizon = Vec3f(0.055f, 0.082f, 0.165f);
    } else if (goldenSunset) {
        outZenith  = Vec3f(0.14f, 0.24f, 0.52f);
        outHorizon = Vec3f(0.86f, 0.54f, 0.34f);
    } else {
        outZenith  = Vec3f(0.12f, 0.36f, 0.74f);
        outHorizon = Vec3f(0.58f, 0.76f, 0.92f);
    }
}

struct SkyStar {
    Vec3f dir;
    float r, g, b;
    float phase;
    bool isBright;
};

static std::vector<SkyStar> g_nightStars;

void initNightStars() {
    if (!g_nightStars.empty()) return;
    g_nightStars.reserve(2600);
    uint32_t seed = 987654321U;
    auto nextRand = [&]() -> float {
        seed = seed * 1664525U + 1013904223U;
        return (float)(seed & 0xFFFF) / 65535.0f;
    };

    for (int i = 0; i < 2600; ++i) {
        float theta = nextRand() * 2.0f * (float)GeoProjection::PI;
        float u = nextRand();
        float y = 0.04f + 0.96f * std::pow(u, 0.75f);
        float rad = std::sqrt(std::max(0.0f, 1.0f - y * y));
        Vec3f dir(std::cos(theta) * rad, y, std::sin(theta) * rad);

        float cPick = nextRand();
        float r = 0.95f, g = 0.96f, b = 1.0f;
        if (cPick < 0.22f)      { r = 1.00f; g = 0.88f; b = 0.72f; }
        else if (cPick < 0.45f) { r = 0.78f; g = 0.88f; b = 1.00f; }

        g_nightStars.push_back({dir, r, g, b, nextRand() * 6.28f, (i % 7 == 0)});
    }
}

void drawSkyDomeAndNightMoon(const Vec3f& camPos, const Vec3f& zenCol, const Vec3f& horCol,
                             bool nightMode, bool goldenSunset, float simTime) {
    initNightStars();

    glPushMatrix();
    glTranslatef(camPos.x, camPos.y, camPos.z);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    const int SLICES = 48;
    const float R = 20000.0f;

    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= SLICES; ++i) {
        float a = i * 2.0f * (float)GeoProjection::PI / SLICES;
        float cx = std::sin(a) * R, cz = std::cos(a) * R;
        glColor3f(horCol.x * 0.85f, horCol.y * 0.88f, horCol.z * 0.92f);
        glVertex3f(cx, -12000.0f, cz);
        glColor3f(horCol.x, horCol.y, horCol.z);
        glVertex3f(cx, 0.0f, cz);
    }
    glEnd();

    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= SLICES; ++i) {
        float a = i * 2.0f * (float)GeoProjection::PI / SLICES;
        float cx = std::sin(a) * R, cz = std::cos(a) * R;
        glColor3f(horCol.x, horCol.y, horCol.z);
        glVertex3f(cx, 0.0f, cz);
        glColor3f(zenCol.x, zenCol.y, zenCol.z);
        glVertex3f(cx * 0.40f, 8500.0f, cz * 0.40f);
    }
    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    glColor3f(zenCol.x, zenCol.y, zenCol.z);
    glVertex3f(0.0f, 14000.0f, 0.0f);
    for (int i = 0; i <= SLICES; ++i) {
        float a = i * 2.0f * (float)GeoProjection::PI / SLICES;
        float cx = std::sin(a) * R * 0.40f, cz = std::cos(a) * R * 0.40f;
        glVertex3f(cx, 8500.0f, cz);
    }
    glEnd();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (nightMode) {
        glPointSize(1.5f);
        glBegin(GL_POINTS);
        for (const auto& st : g_nightStars) {
            if (st.isBright) continue;
            float twinkle = 0.65f + 0.35f * std::sin(simTime * 2.4f + st.phase);
            float horizonFade = std::clamp((st.dir.y - 0.04f) * 4.0f, 0.0f, 1.0f);
            glColor4f(st.r, st.g, st.b, twinkle * horizonFade * 0.88f);
            glVertex3f(st.dir.x * 15000.0f, st.dir.y * 15000.0f, st.dir.z * 15000.0f);
        }
        glEnd();

        glPointSize(3.0f);
        glBegin(GL_POINTS);
        for (const auto& st : g_nightStars) {
            if (!st.isBright) continue;
            float twinkle = 0.72f + 0.28f * std::sin(simTime * 3.2f + st.phase);
            float horizonFade = std::clamp((st.dir.y - 0.04f) * 4.0f, 0.0f, 1.0f);
            glColor4f(st.r, st.g, st.b, twinkle * horizonFade);
            glVertex3f(st.dir.x * 15000.0f, st.dir.y * 15000.0f, st.dir.z * 15000.0f);
        }
        glEnd();
        glPointSize(1.0f);
    }

    Vec3f celestialDir = nightMode ? normalize3(Vec3f(-0.45f, 0.46f, 0.42f))
                                   : (goldenSunset ? normalize3(Vec3f(-0.82f, 0.16f, 0.35f))
                                                   : normalize3(Vec3f(-0.45f, 0.68f, 0.42f)));
    Vec3f cPos = celestialDir * 14000.0f;
    Vec3f rightAxis = normalize3(cross3(celestialDir, Vec3f(0.0f, 1.0f, 0.0f)));
    Vec3f upAxis    = normalize3(cross3(rightAxis, celestialDir));

    auto drawDisc = [&](float radius, float r, float g, float b, float centerAlpha, float edgeAlpha,
                        float offRight = 0.0f, float offUp = 0.0f) {
        Vec3f c = cPos + rightAxis * offRight + upAxis * offUp;
        glBegin(GL_TRIANGLE_FAN);
        glColor4f(r, g, b, centerAlpha);
        glVertex3f(c.x, c.y, c.z);
        glColor4f(r, g, b, edgeAlpha);
        const int MSEG = 36;
        for (int i = 0; i <= MSEG; ++i) {
            float a = (i / (float)MSEG) * 2.0f * (float)GeoProjection::PI;
            Vec3f pt = c + rightAxis * (std::cos(a) * radius) + upAxis * (std::sin(a) * radius);
            glVertex3f(pt.x, pt.y, pt.z);
        }
        glEnd();
    };

    if (nightMode) {
        drawDisc(2400.0f, 0.45f, 0.62f, 0.95f, 0.16f, 0.0f);
        drawDisc(1050.0f, 0.75f, 0.86f, 1.00f, 0.32f, 0.0f);
        drawDisc(440.0f,  1.00f, 0.98f, 0.90f, 1.00f, 0.95f);
        drawDisc(135.0f,  0.80f, 0.82f, 0.78f, 0.55f, 0.0f, -110.0f,  75.0f);
        drawDisc(110.0f,  0.78f, 0.80f, 0.76f, 0.50f, 0.0f,   85.0f,  45.0f);
        drawDisc( 95.0f,  0.82f, 0.83f, 0.79f, 0.45f, 0.0f,   30.0f, -95.0f);
    } else if (goldenSunset) {
        drawDisc(3600.0f, 1.00f, 0.45f, 0.15f, 0.28f, 0.0f);
        drawDisc(1400.0f, 1.00f, 0.72f, 0.32f, 0.48f, 0.0f);
        drawDisc(520.0f,  1.00f, 0.92f, 0.72f, 1.00f, 0.92f);
    } else {
        drawDisc(2200.0f, 1.00f, 0.95f, 0.80f, 0.22f, 0.0f);
        drawDisc(420.0f,  1.00f, 0.99f, 0.92f, 0.98f, 0.90f);
    }

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix();
}

// 3D Kaza Patlama / Alev Küresi / Enkaz ve Duman Efekti!
void renderCrashExplosion3D(const Vec3f& crashPos, float crashTimer) {
    glUseProgram(0);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (int i = 0; i < 24; ++i) {
        float seed = i * 1.732f;
        float vx = std::sin(seed * 3.1f) * (6.0f + (i % 5) * 3.5f);
        float vz = std::cos(seed * 4.7f) * (6.0f + (i % 4) * 3.5f);
        float vy = 8.0f + (i % 6) * 4.0f;

        float t = std::fmod(crashTimer + (i * 0.08f), 2.5f);
        float px = crashPos.x + vx * t;
        float py = crashPos.y + vy * t - 2.5f * t * t;
        float pz = crashPos.z + vz * t;
        float sz = 1.6f + t * 2.2f;

        glPushMatrix();
        glTranslatef(px, py, pz);
        glRotatef(t * 140.0f + i * 25.0f, 1.0f, 0.8f, 0.3f);

        if (t < 0.75f) {
            // Parlak Alev Kuresi (Sari -> Turuncu -> Kirmizi)
            glColor4f(1.0f, 0.85f - t * 0.7f, 0.15f, 0.92f);
        } else {
            // Yukselen Siyah/Gri Duman Bulutu
            float fade = std::clamp(1.0f - (t - 0.75f) / 1.75f, 0.0f, 1.0f);
            glColor4f(0.18f, 0.18f, 0.20f, fade * 0.78f);
        }
        DynamicWorldRenderer::drawBox(-sz, -sz, -sz, sz, sz, sz);
        glPopMatrix();
    }
    glEnable(GL_LIGHTING);
}

void renderFloatingStationStarLabels(int winW, int winH, const IzmbWorldData& world,
                                     const Vec3f& camEye, const Vec3f& camTarget,
                                     float simTime, bool isCrashed, const std::string& crashReason) {
    Vec3f f = normalize3(camTarget - camEye);
    Vec3f r = normalize3(cross3(f, Vec3f(0.0f, 1.0f, 0.0f)));
    Vec3f u = cross3(r, f);

    float tanHalfFovY = std::tan(58.0f * (float)GeoProjection::DEG2RAD * 0.5f);
    float aspect = (float)winW / (float)winH;
    float tanHalfFovX = tanHalfFovY * aspect;
    float bobY = std::sin(simTime * 2.6f) * 3.0f;

    glUseProgram(0);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, winW, winH, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    for (const auto& st : world.stations) {
        bool wat = false;
        float gy = world.sampleHeightAt(st.x, st.z, wat);
        Vec3f starPos(st.x, gy + 66.0f + bobY, st.z);
        Vec3f d = starPos - camEye;
        float dist = std::sqrt(dot3(d, d));
        if (dist > 6500.0f || dist < 40.0f) continue;

        float zCam = dot3(d, f);
        if (zCam <= 1.0f) continue;

        float xCam = dot3(d, r);
        float yCam = dot3(d, u);
        float ndcX = (xCam / zCam) / tanHalfFovX;
        float ndcY = (yCam / zCam) / tanHalfFovY;
        if (std::fabs(ndcX) > 0.92f || std::fabs(ndcY) > 0.88f) continue;

        float sx = (ndcX * 0.5f + 0.5f) * winW;
        float sy = (0.5f - ndcY * 0.5f) * winH;

        char label[64];
        std::snprintf(label, sizeof(label), "[%s] %s (%.1fKM)", st.code, st.name, dist / 1000.0f);
        float boxW = std::strlen(label) * 8.2f + 12.0f;
        HudAndRadarRenderer::drawRect(sx - boxW * 0.5f, sy - 10.0f, boxW, 18.0f, 0.03f, 0.08f, 0.16f, 0.82f);
        HudAndRadarRenderer::drawText(sx - boxW * 0.5f + 6.0f, sy - 6.0f, 1.32f, label, 1.0f, 0.92f, 0.28f);
    }

    // Eger Ucak Binaya veya Yere Carptiysa Ortada Kirmizi KAZA / CRASH Ekrani Goster!
    if (isCrashed) {
        float bw = 620.0f, bh = 96.0f;
        float bx = (winW - bw) * 0.5f, by = (winH - bh) * 0.42f;
        HudAndRadarRenderer::drawRect(bx - 4.0f, by - 4.0f, bw + 8.0f, bh + 8.0f, 1.0f, 0.85f, 0.15f, 0.95f);
        HudAndRadarRenderer::drawRect(bx, by, bw, bh, 0.68f, 0.06f, 0.06f, 0.94f);
        HudAndRadarRenderer::drawText(bx + 28.0f, by + 20.0f, 2.35f, crashReason, 1.0f, 0.96f, 0.25f);
        HudAndRadarRenderer::drawText(bx + 28.0f, by + 58.0f, 1.65f, "[R] VEYA [SPACE] VEYA [KOL A]: YENIDEN UCUSA BASLA", 0.98f, 0.98f, 1.0f);
    }

    glEnable(GL_DEPTH_TEST);
}

struct TeleportPreset {
    const char* name;
    double lat, lon;
    float altAGL;
    float headingDeg;
};

static const TeleportPreset PRESETS[10] = {
    { "1: KONAK & SAAT KULESI",          38.41890, 27.12870, 185.0f,  25.0f },
    { "2: ALSANCAK GAR IZBAN",           38.43864, 27.14815, 185.0f,  10.0f },
    { "3: BAYRAKLI & SALHANE IZBAN",     38.45067, 27.17214, 235.0f,  15.0f },
    { "4: KARSIYAKA GAR IZBAN",          38.45788, 27.11514, 185.0f, 290.0f },
    { "5: EGEKENT 2 IZBAN (GERCEK OSM)", 38.56062, 27.04431, 175.0f, 110.0f },
    { "6: ULUKENT IZBAN (GERCEK OSM)",   38.54785, 27.03548, 175.0f,  85.0f },
    { "7: MENEMEN GAR IZBAN",            38.60311, 27.07678, 180.0f, 210.0f },
    { "8: CIGLI & ATASANAYI IZBAN",      38.49164, 27.06341, 185.0f, 330.0f },
    { "9: MAVISEHIR & SEMIKLER IZBAN",   38.48221, 27.08297, 185.0f, 140.0f },
    { "0: ADNAN MENDERES HVL. IZBAN",    38.29132, 27.14799, 175.0f, 345.0f }
};

int main() {
    std::cout << "============================================================================\n";
    std::cout << "  IZMIR 3D SIMULATOR v14.0 (60+ FPS TURBO + 3D BINA CARPISMA & KAZA FIZIGI)\n";
    std::cout << "============================================================================\n";

    IzmbWorldData world;
    if (!world.loadFromBinary("../data/compiled/izmir_world.izmb")) {
        std::cerr << "HATA: ../data/compiled/izmir_world.izmb okunamadi!\n";
        return 1;
    }

    Display* dpy = XOpenDisplay(nullptr);
    if (!dpy) return 1;

    Window root = DefaultRootWindow(dpy);
    GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, GLX_SAMPLE_BUFFERS, 1, GLX_SAMPLES, 4, None };
    XVisualInfo* vi = glXChooseVisual(dpy, 0, att);
    if (!vi) {
        GLint attFallback[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
        vi = glXChooseVisual(dpy, 0, attFallback);
    }

    Colormap cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);
    XSetWindowAttributes swa;
    swa.colormap = cmap;
    swa.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask | StructureNotifyMask;

    int winW = 1280, winH = 720;
    Window win = XCreateWindow(dpy, root, 0, 0, winW, winH, 0, vi->depth, InputOutput,
                               vi->visual, CWColormap | CWEventMask, &swa);
    XMapWindow(dpy, win);
    XStoreName(dpy, win, "IZMIR 3D FLIGHT SIMULATOR v14.0 (BUILDING COLLISION + 60 FPS TURBO)");

    Atom wmDelete = XInternAtom(dpy, "WM_DELETE_WINDOW", True);
    XSetWMProtocols(dpy, win, &wmDelete, 1);

    GLXContext glc = glXCreateContext(dpy, vi, nullptr, GL_TRUE);
    glXMakeCurrent(dpy, win, glc);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_MULTISAMPLE);

    TerrainRenderer terrain;
    if (!terrain.initFromWorldData(world, "../shaders")) return 1;

    WayAndRailRenderer waysAndRails;
    if (!waysAndRails.initFromWorldData(world, "../shaders")) return 1;

    BuildingRenderer buildings;
    if (!buildings.initFromWorldData(world, "../shaders")) return 1;

    DynamicWorldRenderer dynWorld;
    FlightInput input;
    input.tryOpenJoystick();

    GeoProjection proj;
    Vec3f pos(0.0f, 200.0f, 0.0f);
    float yaw = 0.0f, pitch = -0.04f, roll = 0.0f;
    float airspeedMps = 62.0f;
    float propAngle = 0.0f;

    bool goldenSunset = false;
    bool prevG = false;

    // Kaza / Bina Carpisma Durumu
    bool isCrashed = false;
    float crashTimer = 0.0f;
    Vec3f crashPos(0.0f, 0.0f, 0.0f);
    std::string crashReason = "";

    int activeIdx = 2; // Bayrakli & Salhane (Folkart Kuleleri karsisinda basla!)
    auto teleportTo = [&](int idx) {
        activeIdx = idx % 10;
        const auto& p = PRESETS[activeIdx];
        Vec2d xz = proj.latLonToWorldXZ(p.lat, p.lon);
        bool wat = false;
        float gH = world.sampleHeightAt((float)xz.x, (float)xz.z, wat);
        pos = Vec3f((float)xz.x, gH + p.altAGL, (float)xz.z);
        yaw = (float)((180.0 - p.headingDeg) * GeoProjection::DEG2RAD);
        pitch = input.freeCamMode ? -0.18f : -0.02f;
        roll = 0.0f;
        airspeedMps = 62.0f;
        isCrashed = false;
        crashTimer = 0.0f;
    };

    teleportTo(2);

    bool quit = false;
    float simTime = 0.0f;
    float fpsSmooth = 60.0f;
    auto lastTime = std::chrono::steady_clock::now();

    while (!quit) {
        auto frameStart = std::chrono::steady_clock::now();
        while (XPending(dpy) > 0) {
            XEvent xev;
            XNextEvent(dpy, &xev);
            if (xev.type == ConfigureNotify) {
                winW = std::max(320, xev.xconfigure.width);
                winH = std::max(240, xev.xconfigure.height);
                glViewport(0, 0, winW, winH);
            } else if (xev.type == ClientMessage && (Atom)xev.xclient.data.l[0] == wmDelete) {
                quit = true;
            }
        }

        auto now = std::chrono::steady_clock::now();
        float rawDt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        if (rawDt > 0.001f) fpsSmooth = fpsSmooth * 0.90f + (1.0f / rawDt) * 0.10f;
        float dt = std::clamp(rawDt, 0.002f, 0.040f);
        simTime += dt;

        input.update(dpy, dt, quit, activeIdx);
        if (input.requestedTeleport != -1) {
            teleportTo(input.requestedTeleport);
        }

        char keymap[32];
        XQueryKeymap(dpy, keymap);
        auto keyPressed = [&](KeySym sym) -> bool {
            KeyCode kc = XKeysymToKeycode(dpy, sym);
            return (kc != 0) && ((keymap[kc >> 3] & (1 << (kc & 7))) != 0);
        };

        bool nowG = keyPressed(XK_g) || keyPressed(XK_G);
        if (nowG && !prevG) {
            goldenSunset = !goldenSunset;
            if (goldenSunset) input.nightMode = false;
        }
        prevG = nowG;

        Vec3f camEye, camTarget, camFwd;
        float displaySpeedKmh = 0.0f;

        if (isCrashed) {
            crashTimer += dt;
            // Kaza noktasinin etrafinda sinematik kamera donusu
            float orbitAng = yaw + crashTimer * 0.85f;
            camEye    = crashPos + Vec3f(std::sin(orbitAng) * 32.0f, 14.0f + crashTimer * 2.5f, std::cos(orbitAng) * 32.0f);
            camTarget = crashPos + Vec3f(0.0f, 2.5f, 0.0f);
            camFwd    = normalize3(camTarget - camEye);

            // [R], [Space], Oyun Kolu A tusu veya 4 saniye sonra otomatik kurtarma!
            if (keyPressed(XK_r) || keyPressed(XK_R) || keyPressed(XK_space) || input.btn[0] || crashTimer > 4.0f) {
                isCrashed = false;
                crashTimer = 0.0f;
                bool wat = false;
                float gH = world.sampleHeightAt(pos.x, pos.z, wat);
                pos.y = std::max(pos.y, gH) + 145.0f;
                pitch = 0.0f;
                roll  = 0.0f;
                airspeedMps = 62.0f;
            }
        } else if (input.freeCamMode) {
            // SERBEST DRONE MODU (Bina duvarlarina ve catilarina fiziksel carpisma!)
            yaw   -= input.roll * 1.6f * dt;
            pitch -= input.pitch * 1.2f * dt;
            pitch  = std::clamp(pitch, -1.25f, 1.25f);

            float spd = 120.0f + input.throttle * 340.0f;
            camFwd = Vec3f(std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch));
            Vec3f nextPos = pos + camFwd * (spd * dt);
            nextPos.y += input.yaw * 95.0f * dt;

            float hitRoofY = 0.0f;
            Vec3f pushNorm(0.0f, 1.0f, 0.0f);
            if (buildings.checkBuildingCollision(world, nextPos, 2.2f, hitRoofY, pushNorm)) {
                if (pos.y >= hitRoofY - 1.5f) {
                    nextPos.y = hitRoofY + 2.2f; // Catinin ustunde dur
                } else {
                    nextPos = nextPos + pushNorm * 3.5f; // Duvar boyunca kay
                }
            }

            bool wat = false;
            float minG = world.sampleHeightAt(nextPos.x, nextPos.z, wat) + 3.0f;
            if (nextPos.y < minG) nextPos.y = minG;

            pos = nextPos;
            displaySpeedKmh = spd * 3.6f;
            camEye = pos;
            camTarget = pos + camFwd * 50.0f;
        } else {
            // 64-BIT UCAK SIMULASYONU (Gercek 3D Bina ve Arazi Carpisma / Kaza Fizigi!)
            float targetSpeed = (32.0f + input.throttle * 76.0f) * (input.planeModel == 1 ? 1.6f : 1.0f);
            targetSpeed -= std::sin(pitch) * 16.0f;
            airspeedMps += (std::clamp(targetSpeed, 28.0f, 185.0f) - airspeedMps) * 2.5f * dt;

            float desiredRoll  = input.roll * 0.85f;
            float desiredPitch = input.pitch * 0.62f;
            roll  += (desiredRoll - roll) * 4.8f * dt;
            pitch += (desiredPitch - pitch) * 4.2f * dt;

            float turnRate = -std::sin(roll) * 0.95f + input.yaw * 0.60f;
            yaw += turnRate * dt;

            camFwd = Vec3f(std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch));
            pos = pos + camFwd * (airspeedMps * dt);
            propAngle += airspeedMps * 0.65f * dt;
            displaySpeedKmh = airspeedMps * 3.6f;

            // 1. BINA CARPISMA TESTI (Govde + Sol/Sag Kanat Uclari)
            float hitRoofY = 0.0f;
            Vec3f pushNorm;
            if (buildings.checkBuildingCollision(world, pos, 4.2f, hitRoofY, pushNorm)) {
                isCrashed = true;
                crashTimer = 0.0f;
                crashPos = pos;
                crashReason = "KAZA! BINAYA CARPTINIZ! (BUILDING HIT)";
            }

            // 2. ARAZI / DENIZ CARPISMA TESTI
            bool wat = false;
            float groundY = world.sampleHeightAt(pos.x, pos.z, wat);
            if (!isCrashed && pos.y < groundY + 2.2f) {
                if (pitch < -0.22f || std::fabs(roll) > 0.48f || wat) {
                    isCrashed = true;
                    crashTimer = 0.0f;
                    pos.y = groundY + 1.0f;
                    crashPos = pos;
                    crashReason = wat ? "KAZA! KORFEZ SULARINA CAKILDINIZ!" : "KAZA! SERT ARAZI CARPISMASI!";
                } else {
                    pos.y = groundY + 2.2f;
                    pitch = std::max(0.0f, pitch);
                    roll *= 0.85f;
                }
            }

            if (input.camMode == 0) {
                camEye    = pos - camFwd * 16.5f + Vec3f(0.0f, 5.5f, 0.0f);
                camTarget = pos + camFwd * 20.0f;
            } else {
                camEye    = pos + camFwd * 1.8f + Vec3f(0.0f, 0.6f, 0.0f);
                camTarget = pos + camFwd * 50.0f;
            }
        }

        Vec3f zenCol, fogCol;
        computeSkyColors(input.nightMode, goldenSunset, zenCol, fogCol);

        glClearColor(fogCol.x, fogCol.y, fogCol.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        setPerspective(58.0, (double)winW / (double)winH, 2.0, 26000.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        setLookAt(camEye, camTarget, Vec3f(0.0f, 1.0f, 0.0f));

        drawSkyDomeAndNightMoon(camEye, zenCol, fogCol, input.nightMode, goldenSunset, simTime);

        Vec3f sunDir = input.nightMode ? normalize3(Vec3f(-0.45f, 0.46f, 0.42f))
                                       : (goldenSunset ? normalize3(Vec3f(-0.82f, 0.22f, 0.35f))
                                                       : normalize3(Vec3f(-0.45f, 0.68f, 0.42f)));
        GLfloat lightPos[4] = { sunDir.x, sunDir.y, sunDir.z, 0.0f };
        glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

        terrain.render(simTime, input.nightMode, sunDir, camEye, camFwd, fogCol);
        waysAndRails.render(input.nightMode, sunDir, camEye, camFwd, fogCol);
        buildings.render(input.nightMode, sunDir, camEye, camFwd, fogCol);
        dynWorld.renderDynamicVehicles(world, proj, simTime, input.nightMode || goldenSunset, camEye);

        if (isCrashed) {
            renderCrashExplosion3D(crashPos, crashTimer);
        } else if (!input.freeCamMode && input.camMode == 0) {
            glPushMatrix();
            glTranslatef(pos.x, pos.y, pos.z);
            glRotatef(yaw * 57.29578f, 0, 1, 0);
            glRotatef(-pitch * 57.29578f, 1, 0, 0);
            glRotatef(-roll * 57.29578f, 0, 0, 1);
            DynamicWorldRenderer::renderAircraft3D(input.planeModel, propAngle, input.roll, input.pitch);
            glPopMatrix();
        }

        renderFloatingStationStarLabels(winW, winH, world, camEye, camTarget, simTime, isCrashed, crashReason);

        GeoCoord curLL = proj.worldXZToLatLon(pos.x, pos.z);
        HudAndRadarRenderer::renderHudAndVectorRadar(
            winW, winH, world, dynWorld, input, pos, yaw, pitch,
            displaySpeedKmh, fpsSmooth, PRESETS[activeIdx].name, curLL.lat, curLL.lon);

        glXSwapBuffers(dpy, win);

        // Sadece kare 8ms'den daha hizli bittiyse (~120 FPS ustu) islemciyi yormamak icin cok kisa bekle
        auto frameEnd = std::chrono::steady_clock::now();
        float elapsedMs = std::chrono::duration<float, std::milli>(frameEnd - frameStart).count();
        if (elapsedMs < 6.0f) {
            std::this_thread::sleep_for(std::chrono::milliseconds((int)(6.0f - elapsedMs)));
        }
    }

    glXMakeCurrent(dpy, None, nullptr);
    glXDestroyContext(dpy, glc);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
    return 0;
}
