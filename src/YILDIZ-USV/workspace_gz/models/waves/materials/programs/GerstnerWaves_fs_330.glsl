#version 330

uniform sampler2D bumpMap;
uniform samplerCube cubeMap;

uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

// receive height from vertex shader (for optional foam)
in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  float height;
} inPs;

out vec4 fragColor;

void main()
{
  vec4 bump = texture(bumpMap, inPs.bumpCoord)*2.0 - 1.0;
  vec3 N = normalize(inPs.rotMatrix * bump.xyz);

  vec3 E = normalize(inPs.eyeVec);
  vec3 R = reflect(E, N);

  R = vec3(R.x, R.y, -R.z);

  vec4 envColor = texture(cubeMap, R, 0.0);

  envColor.rgb *= (envColor.r+envColor.g+envColor.b)*hdrMultiplier;

  float facing = 1.0 - dot(-E, N);
  float waterEnvRatio = clamp(pow(facing, fresnelPower), 0.05, 1.0);

  vec4 waterColor = mix(shallowColor, deepColor, facing);

  vec4 color = mix(waterColor, envColor, waterEnvRatio);

  // Optional simple foam effect: small white at high local height
  float foam = smoothstep(0.2, 0.6, inPs.height); // thresholds ayarla
  color = mix(color, vec4(1.0,1.0,1.0,1.0), foam * 0.6); // foam yoğunluğunu çarpanla

  fragColor = vec4(color.xyz, 1.0);
}
