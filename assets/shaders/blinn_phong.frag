#version 460 core

in vec3 vFragPos;
in vec3 vNormal;

uniform vec3  uViewPos;
uniform vec3  uLightDir;
uniform vec3  uLightColor;
uniform vec3  uAlbedo;
uniform float uShininess;

out vec4 FragColor;

void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(-uLightDir);        // 光照方向 -> 入射方向
    vec3 V = normalize(uViewPos - vFragPos);
    vec3 H = normalize(L + V);             // Blinn-Phong 半程向量

    vec3 ambient  = 0.15 * uLightColor;
    float diff    = max(dot(N, L), 0.0);
    vec3 diffuse  = diff * uLightColor;
    float spec    = pow(max(dot(N, H), 0.0), uShininess);
    vec3 specular = spec * uLightColor;

    vec3 result = (ambient + diffuse) * uAlbedo + specular;
    FragColor = vec4(result, 1.0);
}
