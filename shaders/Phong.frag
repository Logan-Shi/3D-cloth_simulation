#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

vec4 shadePhong() {
  float p = 8.0;

  vec4 color = u_color * 0.35;

  vec3 lightVec = u_light_pos - v_position.xyz;
  vec3 lightDir = normalize(lightVec);
  vec3 outDir = normalize(u_cam_pos - v_position.xyz);
  vec3 n = normalize(v_normal.xyz);

  float distFactor = 1.0 / sqrt(dot(lightVec, lightVec));

  vec4 ambient = color * 0.9;
  // ambient.a = 0.5;

  float diffuseDot = dot(n, lightDir);
  vec4 diffuse = color * clamp(diffuseDot, 0.0, 1.0);

  vec3 halfAngle = normalize(outDir + lightDir);
  vec4 specularColor = min(color + 0.2, 1.0);
  float specularDot = dot(n, halfAngle);
  vec4 specular = 0.5 * specularColor * pow(clamp(specularDot, 0.0, 1.0), p);

  return diffuse + ambient + specular;
}

void main() {
  out_color = shadePhong();
  out_color.a = 0.5;
}