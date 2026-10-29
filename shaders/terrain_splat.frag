#version 120

uniform sampler2D uNoiseTex;
uniform sampler2D uRoadMapTex; // 4096x4096 Tum Izmir Vektor Yol, Kavsak & Sehir Haritasi!
uniform float uTime;
uniform float uNightMode;
uniform vec3 uSunDir;
uniform vec3 uCameraPos;
uniform vec3 uFogColor;

varying vec3 vWorldPos;
varying vec3 vNormal;
varying vec4 vWeights;

float sampleFBM(vec2 p) {
    float v = 0.0;
    v += texture2D(uNoiseTex, p * 0.00045).r * 0.50;
    v += texture2D(uNoiseTex, p * 0.00210).g * 0.28;
    v += texture2D(uNoiseTex, p * 0.01150).b * 0.14;
    v += texture2D(uNoiseTex, p * 0.06500).r * 0.08;
    return v;
}

vec3 acesToneMap(vec3 x) {
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

void main() {
    vec3 viewVec = uCameraPos - vWorldPos;
    float dist = length(viewVec);
    vec3 viewDir = viewVec / max(dist, 0.001);

    float edgeDistX = max(max(-14500.0 - vWorldPos.x, vWorldPos.x - 11500.0), 0.0);
    float edgeDistZ = max(max(-22500.0 - vWorldPos.z, vWorldPos.z - 17500.0), 0.0);
    float borderFade = clamp(1.0 - max(edgeDistX, edgeDistZ) / 1500.0, 0.0, 1.0);
    float fogFactor  = clamp(exp(-dist * 0.000085), 0.0, 1.0) * borderFade;

    float sunViewDot = max(dot(-viewDir, uSunDir), 0.0);
    vec3 atmoColor = uFogColor + vec3(0.32, 0.18, 0.06) * pow(sunViewDot, 6.0) * (1.0 - uNightMode * 0.8);

    float fbmVal    = sampleFBM(vWorldPos.xz);
    float fineGrain = texture2D(uNoiseTex, vWorldPos.xz * 0.25).b;
    float microBump = texture2D(uNoiseTex, vWorldPos.xz * 0.08).r - 0.5;

    float waterMask = vWeights.r;

    // 1. IZMIR KORFEZI DENIZ FIZIGI
    if (waterMask > 0.50 && vWorldPos.y < 1.2) {
        vec2 wUv1 = vWorldPos.xz * 0.009 + vec2(uTime * 0.026, uTime * 0.018);
        vec2 wUv2 = vWorldPos.xz * 0.031 - vec2(uTime * 0.021, uTime * 0.032);
        vec3 n1 = texture2D(uNoiseTex, wUv1).rgb - 0.5;
        vec3 n2 = texture2D(uNoiseTex, wUv2).rgb - 0.5;
        vec3 waterNorm = normalize(vec3((n1.x + n2.y * 0.7) * 0.26, 1.0, (n1.y - n2.x * 0.7) * 0.26));

        float fresnel = pow(1.0 - max(dot(waterNorm, viewDir), 0.0), 3.5);
        float shallow = clamp((0.80 - waterMask) * 3.8, 0.0, 1.0);

        vec3 seaCol = mix(vec3(0.02, 0.14, 0.29), vec3(0.06, 0.34, 0.44), shallow);
        seaCol = mix(seaCol, atmoColor * 0.92, clamp(fresnel * 0.58, 0.0, 0.85));
        if (uNightMode > 0.5) seaCol *= 0.18;

        vec3 halfVec = normalize(uSunDir + viewDir);
        float specMain = pow(max(dot(waterNorm, halfVec), 0.0), 140.0);
        vec3 sunColor = (uSunDir.y < 0.32 && uNightMode < 0.5) ? vec3(1.0, 0.62, 0.28) : vec3(1.0, 0.95, 0.84);
        seaCol += sunColor * specMain * (uNightMode > 0.5 ? 0.25 : 1.25);

        seaCol = acesToneMap(seaCol * 1.10);
        gl_FragColor = vec4(mix(atmoColor, seaCol, fogFactor), 1.0);
        return;
    }

    // 2. KARA ZEMINI + 4096x4096 VEKTOR YOL & YAPI ADASI HARITASI (uRoadMapTex)
    vec3 n = normalize(vNormal + vec3(microBump * 0.16, 0.0, (fineGrain - 0.5) * 0.16));
    float slope = 1.0 - clamp(vNormal.y, 0.0, 1.0);
    float micro = 0.84 + 0.32 * (fineGrain - 0.5);

    vec3 lushOlive  = vec3(0.20, 0.35, 0.16);
    vec3 dryMeadow  = vec3(0.42, 0.38, 0.26);
    vec3 lowLandCol = mix(lushOlive, dryMeadow, smoothstep(0.30, 0.70, fbmVal)) * micro;

    vec3 pineForest = vec3(0.14, 0.27, 0.12) * micro;
    float forestBand = smoothstep(55.0, 240.0, vWorldPos.y) * (1.0 - smoothstep(580.0, 880.0, vWorldPos.y));
    vec3 natureCol  = mix(lowLandCol, pineForest, forestBand * 0.78);

    vec3 aegeanRock = mix(vec3(0.44, 0.41, 0.37), vec3(0.57, 0.52, 0.46), fbmVal) * micro;
    float rockWeight = clamp((slope - 0.05) * 8.0 + smoothstep(500.0, 850.0, vWorldPos.y) * 0.7, 0.0, 1.0);
    vec3 baseCol = mix(natureCol, aegeanRock, rockWeight);

    // Dunya Koordinatindan 4096x4096 Harita UV'si:
    // r: Yol Asfalt Yogunlugu, g: Yapi Adasi / Parsel Beton Tabani, b: Bulvar/Otoyol Serit & Ray Vurgusu
    vec2 mapUv = vec2((vWorldPos.x - (-15000.0)) / 27000.0, (vWorldPos.z - (-23000.0)) / 41000.0);
    vec4 roadMap = texture2D(uRoadMapTex, mapUv);

    float urbanW = clamp(max(vWeights.g, roadMap.g) * (1.0 - rockWeight * 0.80), 0.0, 0.95);
    vec3 urbanConcrete = mix(vec3(0.50, 0.49, 0.46), vec3(0.58, 0.56, 0.52), fbmVal) * (0.92 + 0.16 * (fineGrain - 0.5));
    baseCol = mix(baseCol, urbanConcrete, urbanW);

    // 4096x4096 Zemin Asfalt Yol Agi: Hicbir yol yamacta veya uzakta KESILEMEZ!
    if (roadMap.r > 0.08) {
        vec3 roadAsphalt = mix(vec3(0.25, 0.26, 0.29), vec3(0.18, 0.19, 0.22), roadMap.b) * (0.90 + 0.20 * (fineGrain - 0.5));
        if (roadMap.b > 0.55) {
            roadAsphalt = mix(roadAsphalt, vec3(0.85, 0.78, 0.45), (roadMap.b - 0.55) * 1.2);
        }
        baseCol = mix(baseCol, roadAsphalt, clamp(roadMap.r * 1.35, 0.0, 0.98));
    }

    float ndotl = max(dot(n, uSunDir), 0.0);
    vec3 skyAmb = (uNightMode > 0.5) ? vec3(0.08, 0.11, 0.18) : vec3(0.38, 0.43, 0.50);
    vec3 sunCol = (uSunDir.y < 0.32 && uNightMode < 0.5) ? vec3(1.05, 0.72, 0.42) : ((uNightMode > 0.5) ? vec3(0.18, 0.22, 0.32) : vec3(1.00, 0.97, 0.90));
    vec3 lit = baseCol * (skyAmb + sunCol * ndotl * 0.72);

    // Gece Modunda Tum Sokak Agi Boyunca Kesintisiz Sokak Lambasi Isigi!
    if (uNightMode > 0.5 && (roadMap.r > 0.12 || urbanW > 0.25)) {
        float lampGlow = roadMap.r * 0.85 + urbanW * 0.22;
        lit += vec3(1.0, 0.74, 0.34) * lampGlow * 0.55;
    }

    lit = acesToneMap(lit * 1.06);
    gl_FragColor = vec4(mix(atmoColor, lit, fogFactor), 1.0);
}
