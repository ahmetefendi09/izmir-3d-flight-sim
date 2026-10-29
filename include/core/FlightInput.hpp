#pragma once
#include <string>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>

namespace izmir {

class FlightInput {
public:
    int jsFd = -1;
    std::string deviceName = "KLAVYE (JOYSTICK YOK)";
    float axis[8] = {0.0f};
    float axisInit[8] = {0.0f};
    bool axisCalib[8] = {false};
    bool axisMoved[8] = {false};
    bool btn[16] = {false};
    int reconnectTimer = 0;

    float kbRoll = 0.0f, kbPitch = 0.0f, kbYaw = 0.0f;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    float throttle = 0.65f;

    bool freeCamMode = false;
    int planeModel = 0;
    int camMode = 0;
    bool nightMode = false;
    bool radarWide = false;
    int requestedTeleport = -1;

private:
    bool prevF = false, prevV = false, prevC = false, prevT = false, prevZ = false;
    bool prevN = false, prevP = false;

    static float cubicCurve(float v, float deadzone = 0.07f) {
        if (std::fabs(v) <= deadzone) return 0.0f;
        float sign = (v > 0.0f) ? 1.0f : -1.0f;
        float u = (std::fabs(v) - deadzone) / (1.0f - deadzone);
        u = std::clamp(u, 0.0f, 1.0f);
        return sign * (0.45f * u + 0.55f * u * u * u);
    }

    float getTriggerVal(int idx) const {
        if (!axisMoved[idx]) return 0.0f;
        if (axisInit[idx] < -0.35f) {
            return std::clamp((axis[idx] + 1.0f) * 0.5f, 0.0f, 1.0f);
        }
        return std::clamp(std::fabs(axis[idx] - axisInit[idx]), 0.0f, 1.0f);
    }

public:
    void tryOpenJoystick() {
        if (jsFd >= 0) return;
        const char* devs[] = {"/dev/input/js0", "/dev/input/js1", "/dev/input/js2", "/dev/input/js3"};
        for (const char* d : devs) {
            int fd = open(d, O_RDONLY | O_NONBLOCK);
            if (fd >= 0) {
                jsFd = fd;
                char nameBuf[128] = "GAMEPAD/WHEEL";
                if (ioctl(jsFd, JSIOCGNAME(sizeof(nameBuf)), nameBuf) >= 0) {
                    deviceName = std::string(nameBuf);
                    if (deviceName.size() > 22) deviceName = deviceName.substr(0, 22);
                } else {
                    deviceName = "GAMEPAD (JS0)";
                }
                for (int i = 0; i < 8; ++i) {
                    axis[i] = 0.0f;
                    axisInit[i] = 0.0f;
                    axisCalib[i] = false;
                    axisMoved[i] = false;
                }
                for (int i = 0; i < 16; ++i) btn[i] = false;
                break;
            }
        }
    }

    void update(Display* dpy, float dt, bool& quitFlag, int currentPreset) {
        requestedTeleport = -1;

        // 1. LINUX JOYSTICK / DIREKSIYON / GAMEPAD OKUMA (/dev/input/js0)
        if (jsFd < 0) {
            if (++reconnectTimer > 60) {
                reconnectTimer = 0;
                tryOpenJoystick();
            }
        } else {
            struct js_event e;
            while (read(jsFd, &e, sizeof(e)) > 0) {
                bool isInit = (e.type & JS_EVENT_INIT) != 0;
                uint8_t type = e.type & ~JS_EVENT_INIT;

                if (type == JS_EVENT_AXIS && e.number < 8) {
                    float v = std::clamp(e.value / 32767.0f, -1.0f, 1.0f);
                    if (isInit || !axisCalib[e.number]) {
                        axisInit[e.number] = v;
                        axisCalib[e.number] = true;
                    } else if (std::fabs(v - axisInit[e.number]) > 0.08f) {
                        axisMoved[e.number] = true;
                    }
                    axis[e.number] = v;
                } else if (type == JS_EVENT_BUTTON && e.number < 16) {
                    bool pressed = (e.value != 0);
                    if (!isInit && pressed && !btn[e.number]) {
                        // Oyun Kolu / Direksiyon Tus Atamalari:
                        if (e.number == 1) nightMode = !nightMode;                          // B / Daire: Gece-Gunduz
                        if (e.number == 2) freeCamMode = !freeCamMode;                      // X / Kare: Ucak <-> Drone Modu
                        if (e.number == 3) requestedTeleport = (currentPreset + 1) % 10;    // Y / Ucgen: Sonraki Bolge
                        if (e.number == 6) planeModel = (planeModel + 1) % 3;               // Select/Back: Ucak Degistir
                        if (e.number == 7) camMode = (camMode + 1) % 2;                     // Start: Kamera Degistir
                    }
                    btn[e.number] = pressed;
                }
            }
            if (errno != EAGAIN && errno != 0) {
                close(jsFd);
                jsFd = -1;
                deviceName = "KLAVYE (KOL CIKARILDI)";
            }
        }

        float jRoll = 0.0f, jPitch = 0.0f, jYaw = 0.0f;
        if (jsFd >= 0) {
            jRoll = cubicCurve(axis[0], 0.07f);
            if (axisMoved[1]) {
                jPitch = cubicCurve(axis[1], 0.07f);
            }
            // Sag analog veya direksiyon pedallari / L1-R1 omuz tuslari
            if (btn[5]) jYaw -= 1.0f; // R1: Saga Yaw
            if (btn[4]) jYaw += 1.0f; // L1: Sola Yaw

            float accel = std::max(getTriggerVal(5), btn[0] ? 1.0f : 0.0f); // R2 / Gaz Pedali veya A(X) tusu
            float brake = getTriggerVal(2);                                  // L2 / Fren Pedali
            throttle = std::clamp(throttle + (accel - brake) * 0.55f * dt, 0.15f, 1.0f);
        }

        // 2. KLAVYE OKUMA
        char keymap[32];
        XQueryKeymap(dpy, keymap);
        auto keyDown = [&](KeySym sym) -> bool {
            KeyCode code = XKeysymToKeycode(dpy, sym);
            return (code != 0) && ((keymap[code >> 3] & (1 << (code & 7))) != 0);
        };

        if (keyDown(XK_Escape)) quitFlag = true;
        if (keyDown(XK_1)) requestedTeleport = 0;
        if (keyDown(XK_2)) requestedTeleport = 1;
        if (keyDown(XK_3)) requestedTeleport = 2;
        if (keyDown(XK_4)) requestedTeleport = 3;
        if (keyDown(XK_5)) requestedTeleport = 4;
        if (keyDown(XK_6)) requestedTeleport = 5;
        if (keyDown(XK_7)) requestedTeleport = 6;
        if (keyDown(XK_8)) requestedTeleport = 7;
        if (keyDown(XK_9)) requestedTeleport = 8;
        if (keyDown(XK_0)) requestedTeleport = 9;

        bool nowN = keyDown(XK_n) || keyDown(XK_N) || keyDown(XK_Tab);
        if (nowN && !prevN) requestedTeleport = (currentPreset + 1) % 10;
        prevN = nowN;

        bool nowP = keyDown(XK_p) || keyDown(XK_P);
        if (nowP && !prevP) requestedTeleport = (currentPreset + 9) % 10;
        prevP = nowP;

        bool nowF = keyDown(XK_f) || keyDown(XK_F);
        if (nowF && !prevF) freeCamMode = !freeCamMode;
        prevF = nowF;

        bool nowV = keyDown(XK_v) || keyDown(XK_V);
        if (nowV && !prevV) planeModel = (planeModel + 1) % 3;
        prevV = nowV;

        bool nowC = keyDown(XK_c) || keyDown(XK_C);
        if (nowC && !prevC) camMode = (camMode + 1) % 2;
        prevC = nowC;

        bool nowT = keyDown(XK_t) || keyDown(XK_T);
        if (nowT && !prevT) nightMode = !nightMode;
        prevT = nowT;

        bool nowZ = keyDown(XK_z) || keyDown(XK_Z);
        if (nowZ && !prevZ) radarWide = !radarWide;
        prevZ = nowZ;

        if (keyDown(XK_Shift_L)   || keyDown(XK_Shift_R))   throttle = std::min(1.0f, throttle + 0.50f * dt);
        if (keyDown(XK_Control_L) || keyDown(XK_Control_R)) throttle = std::max(0.15f, throttle - 0.50f * dt);

        bool kLeft  = keyDown(XK_a) || keyDown(XK_A) || keyDown(XK_Left);
        bool kRight = keyDown(XK_d) || keyDown(XK_D) || keyDown(XK_Right);
        bool kUp    = keyDown(XK_s) || keyDown(XK_S) || keyDown(XK_Down);
        bool kDown  = keyDown(XK_w) || keyDown(XK_W) || keyDown(XK_Up);
        bool kYawL  = keyDown(XK_q) || keyDown(XK_Q);
        bool kYawR  = keyDown(XK_e) || keyDown(XK_E);

        if (kLeft)       kbRoll = std::max(-1.0f, kbRoll - 2.6f * dt);
        else if (kRight) kbRoll = std::min( 1.0f, kbRoll + 2.6f * dt);
        else             kbRoll *= std::max(0.0f, 1.0f - 5.0f * dt);

        if (kUp)         kbPitch = std::min( 1.0f, kbPitch + 2.4f * dt);
        else if (kDown)  kbPitch = std::max(-1.0f, kbPitch - 2.4f * dt);
        else             kbPitch *= std::max(0.0f, 1.0f - 5.0f * dt);

        if (kYawL)       kbYaw = std::min( 1.0f, kbYaw + 2.2f * dt);
        else if (kYawR)  kbYaw = std::max(-1.0f, kbYaw - 2.2f * dt);
        else             kbYaw *= std::max(0.0f, 1.0f - 5.5f * dt);

        // Hangisi daha buyukse (Oyun Kolu veya Klavye) onu aktif olarak kullan!
        roll  = (std::fabs(jRoll)  > std::fabs(kbRoll))  ? jRoll  : kbRoll;
        pitch = (std::fabs(jPitch) > std::fabs(kbPitch)) ? jPitch : kbPitch;
        yaw   = (std::fabs(jYaw)   > std::fabs(kbYaw))   ? jYaw   : kbYaw;
    }
};

} // namespace izmir
