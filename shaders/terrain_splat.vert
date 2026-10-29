#version 120

varying vec3 vWorldPos;
varying vec3 vNormal;
varying vec4 vWeights; // r: Su Maskesi, g: Sehir Yogunlugu, b: Kiyi Yakinligi, a: Rakim Faktoru

void main() {
    vWorldPos = gl_Vertex.xyz;
    vNormal   = gl_Normal;
    vWeights  = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
