#version 120

uniform float uNightMode;
uniform vec3 uSunDir;
uniform vec3 uCameraPos;
uniform vec3 uFogColor;

varying vec3 vWorldPos;
varying vec3 vNormal;
varying vec2 vUv;
varying vec4 vColor;

float hash21(vec2 p) {
    p = fract(p * vec2(127.1, 311.7));
    p += dot(p, p + 34.17);
    return fract(p.x * p.y);
}

void main() {
    vec3 viewVec = uCameraPos - vWorldPos;
    float dist = length(viewVec);
    vec3 viewDir = viewVec / max(dist, 0.001);

    float fogFactor = clamp(exp(-dist * 0.000085), 0.0, 1.0);
    float sunViewDot = max(dot(-viewDir, uSunDir), 0.0);
    vec3 atmoColor = uFogColor + vec3(0.32, 0.18, 0.06) * pow(sunViewDot, 6.0) * (1.0 - uNightMode * 0.8);

    int cat = int(vColor.a * 10.0 + 0.5);
    float u = vUv.x;
    float v = vUv.y;
    float absU = abs(u);

    float grain = hash21(floor(vWorldPos.xz * 6.0)) * 0.06 - 0.03;
    vec3 baseCol = vColor.rgb + vec3(grain);

    // Katman 1: Alt Kaldirim/Bordur Gövdesi (cat == 6) -> Kavsaklarda ustteki asfalt bunun ustunu orter!
    if (cat == 6) {
        float curbStripe = step(0.5, fract(v * 0.45));
        baseCol = mix(vec3(0.58, 0.57, 0.55), vec3(0.68, 0.67, 0.64), curbStripe) + vec3(grain);
    }
    // IZBAN Demiryolu Hatti (cat == 4)
    else if (cat == 4) {
        float gravel = hash21(floor(vWorldPos.xz * 10.0)) * 0.14 - 0.07;
        baseCol = vec3(0.38, 0.36, 0.34) + vec3(gravel);
        bool inTrackBed = (absU > 0.12 && absU < 0.78);
        if (inTrackBed && fract(v * 1.50) < 0.42) {
            baseCol = vec3(0.56, 0.55, 0.53) + vec3(gravel * 0.4);
        }
        if (min(abs(absU - 0.28), abs(absU - 0.62)) < 0.045) {
            baseCol = vec3(0.84, 0.88, 0.94);
        }
    }
    // Katman 2: Ust Saf Asfalt Cekirdegi (Kavsak Dairelerinde v < 0.0 oldugu icin serit cizgisi cizilmez -> %100 temiz kavsak!)
    else if (cat <= 3 && v > 0.0) {
        float wear = exp(-pow((absU - 0.45) * 4.5, 2.0)) * 0.04;
        baseCol -= vec3(wear);

        float dash = step(0.48, fract(v * 0.08));
        if (cat == 0) {
            if (absU < 0.04) baseCol = vec3(0.92, 0.72, 0.18);
            else if (abs(absU - 0.48) < 0.024 && dash > 0.5) baseCol = vec3(0.92, 0.93, 0.95);
        } else if (cat == 1) {
            if (absU < 0.028) baseCol = vec3(0.88, 0.89, 0.92);
            else if (abs(absU - 0.48) < 0.022 && dash > 0.5) baseCol = vec3(0.86, 0.87, 0.90);
        } else if (cat == 2) {
            if (absU < 0.025 && dash > 0.5) baseCol = vec3(0.82, 0.83, 0.86);
        }
    }

    vec3 n = normalize(vNormal);
    float ndotl = max(dot(n, uSunDir), 0.0);
    float amb = (uNightMode > 0.5) ? 0.18 : 0.48;
    float dif = (uNightMode > 0.5) ? 0.22 : 0.58;
    vec3 lit = baseCol * (amb + dif * ndotl);

    if (uNightMode > 0.5 && cat <= 2 && v > 0.0) {
        float lampZ = fract(v * 0.03125) - 0.5;
        float pool = exp(-(lampZ * lampZ * 12.0 + u * u * 0.35) * 6.5);
        lit += vec3(1.0, 0.78, 0.38) * pool * 0.70;
    }

    gl_FragColor = vec4(mix(atmoColor, lit, fogFactor), 1.0);
}
