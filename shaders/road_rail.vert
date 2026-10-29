#version 120

varying vec3 vWorldPos;
varying vec3 vNormal;
varying vec2 vUv;    // x: Enine konum (-1..+1), y: Boyuna mesafe (Metre)
varying vec4 vColor; // rgb: Taban Rengi, a: Kategori (0:Otoyol, 1:Bulvar, 2:Cadde, 3:Sokak, 4:IZBAN, 5:Istasyon)

void main() {
    vWorldPos   = gl_Vertex.xyz;
    vNormal     = gl_Normal;
    vUv         = gl_MultiTexCoord0.xy;
    vColor      = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
