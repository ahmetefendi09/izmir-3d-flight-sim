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
    float fogFactor = clamp(exp(-dist * 0.000088), 0.0, 1.0);
    if (fogFactor <= 0.01) {
        gl_FragColor = vec4(uFogColor, 1.0);
        return;
    }

    vec3 n = normalize(vNormal);
    float ndotl = max(dot(n, uSunDir), 0.0);
    vec3 skyAmb = (uNightMode > 0.5) ? vec3(0.10, 0.13, 0.22) : vec3(0.40, 0.45, 0.52);
    vec3 sunCol = (uSunDir.y < 0.32 && uNightMode < 0.5) ? vec3(1.05, 0.70, 0.38) : ((uNightMode > 0.5) ? vec3(0.18, 0.22, 0.32) : vec3(0.98, 0.95, 0.88));

    // Cati Yuzeyi
    if (vUv.x < -0.5) {
        vec3 litRoof = vColor.rgb * (skyAmb + sunCol * ndotl * 0.68) * vColor.a;
        gl_FragColor = vec4(mix(uFogColor, litRoof, fogFactor), 1.0);
        return;
    }

    // PERFORMANS LOD 1: 1800 metreden uzaktaki binalarda agir Parallax/Balkon hesabi yapma!
    if (dist > 1800.0) {
        vec3 fastLit = vColor.rgb * (skyAmb + sunCol * ndotl * 0.65) * clamp(vColor.a, 0.45, 1.0);
        if (uNightMode > 0.5) {
            vec2 cId = floor(vUv);
            if (hash21(cId) < 0.35) fastLit += vec3(0.45, 0.34, 0.14);
        }
        gl_FragColor = vec4(mix(uFogColor, fastLit, fogFactor), 1.0);
        return;
    }

    // YAKIN MESAFE (< 1800m): Tam Mimari Detay (Pencere, Balkon, Zemin Kat Dukkan & Isik)
    vec2 cellId = floor(vUv);
    vec2 fUv    = fract(vUv);
    float unitHash = hash21(cellId + floor(vWorldPos.xz * 0.04));

    bool isSkyscraper = (vColor.b > vColor.r + 0.06);
    vec3 wallCol = vColor.rgb;
    float ao = clamp(vColor.a, 0.36, 1.0);

    vec3 surfaceCol = wallCol;
    float isGlassPane = 0.0;

    if (isSkyscraper) {
        bool mullion = (fUv.x < 0.08 || fUv.x > 0.92 || fUv.y < 0.16 || fUv.y > 0.90);
        surfaceCol = mullion ? (wallCol * 0.45) : (wallCol * (0.52 + 0.15 * unitHash));
        isGlassPane = mullion ? 0.0 : 1.0;
    } else {
        bool floorSlab = (fUv.y < 0.11 || fUv.y > 0.91);
        bool inWin = (fUv.x > 0.20 && fUv.x < 0.80 && fUv.y > 0.24 && fUv.y < 0.82);
        if (floorSlab) {
            surfaceCol = wallCol * 0.78;
        } else if (inWin) {
            bool isFrame = (fUv.x < 0.24 || fUv.x > 0.76 || fUv.y < 0.28 || fUv.y > 0.78 || abs(fUv.x - 0.50) < 0.025);
            if (isFrame) surfaceCol = vec3(0.90, 0.90, 0.92);
            else {
                isGlassPane = 1.0;
                surfaceCol = vec3(0.13, 0.18, 0.25);
            }
        }
    }

    vec3 lit = surfaceCol * (skyAmb + sunCol * ndotl * 0.68) * ao;

    if (isGlassPane > 0.5) {
        if (uNightMode > 0.5) {
            if (unitHash < 0.46) {
                lit = mix(lit, vec3(1.0, 0.84, 0.46), 0.85);
            }
        } else {
            vec3 viewDir = viewVec / max(dist, 0.001);
            vec3 halfV = normalize(uSunDir + viewDir);
            float spec = pow(max(dot(n, halfV), 0.0), 28.0);
            lit = mix(lit, vec3(0.38, 0.58, 0.84), isSkyscraper ? 0.45 : 0.22) + sunCol * spec * 0.55;
        }
    }

    gl_FragColor = vec4(mix(uFogColor, lit, fogFactor), 1.0);
}
