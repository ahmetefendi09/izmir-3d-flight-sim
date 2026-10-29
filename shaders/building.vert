#version 120

varying vec3 vWorldPos;
varying vec3 vNormal;
varying vec2 vUv;      // x: Metre cinsinden yatay cephe, y: Kat sayisi (her 1.0 = 1 kat)
varying vec4 vColor;   // rgb: Bina/Cati rengi, a: MatID + Ambient Occlusion (AO)

void main() {
    vWorldPos = gl_Vertex.xyz;
    vNormal   = gl_Normal;
    vUv       = gl_MultiTexCoord0.xy;
    vColor    = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
