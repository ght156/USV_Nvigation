#version 330

in vec4 vertex;
in vec4 uv0;
uniform mat4 worldviewproj_matrix;

/////////// Input parameters //////////
// Waves
uniform int Nwaves;
uniform vec3 camera_position_object_space;
uniform float rescale;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;
uniform float t;
uniform vec3 amplitude;
uniform vec3 wavenumber;
uniform vec3 omega;
uniform vec3 steepness;
uniform vec2 dir0;
uniform vec2 dir1;
uniform vec2 dir2;
uniform float tau;

// New controls to limit excessive motion
uniform float vertical_scale;    // default ~0.25 (scale vertical displacement)
uniform float horizontal_scale;  // default ~0.6  (scale horizontal displacement)
uniform float max_disp;          // max vertical displacement in meters, default ~0.5

/////////// Output variables to fragment shader //////////
out block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  float height; // pass height for optional foam effects
} outVs;

struct WaveParameters {
  float k;
  float a;
  float omega;
  vec2 d;
  float q;
};

out gl_PerVertex
{
  vec4 gl_Position;
};

void main()
{
  WaveParameters waves[3];

  waves[0] = WaveParameters(wavenumber.x, amplitude.x, omega.x, dir0.xy, steepness.x);
  waves[1] = WaveParameters(wavenumber.y, amplitude.y, omega.y, dir1.xy, steepness.y);
  waves[2] = WaveParameters(wavenumber.z, amplitude.z, omega.z, dir2.xy, steepness.z);

  vec4 P = vertex;

  vec3 B = vec3(1.0, 0.0, 0.0);
  vec3 T = vec3(0.0, 1.0, 0.0);
  vec3 N = vec3(0.0, 0.0, 1.0);

  float totalHeight = 0.0;

  for(int i = 0; i < Nwaves; ++i)
  {
    float k = waves[i].k;
    float a = waves[i].a * (1.0 - exp(-1.0*t/tau));
    float q = waves[i].q;
    float dx = waves[i].d.x;
    float dy = waves[i].d.y;
    float theta = dot(waves[i].d, P.xy)*k - t*waves[i].omega;
    float c = cos(theta);
    float s = sin(theta);

    // scale horizontal displacement to reduce large lateral shifts
    P.x -= horizontal_scale * (q*a*dx*s);
    P.y -= horizontal_scale * (q*a*dy*s);

    // scale vertical displacement to reduce big up/down motion
    float vertDisp = vertical_scale * (a * c);
    P.z += vertDisp;
    totalHeight += vertDisp;

    float ka = a*k;
    float qkac = q*ka*c;
    float kas = ka*s;
    float dxy = dx*dy;

    B += vec3(-qkac*dx*dx, -qkac*dxy, -kas*dx);
    T += vec3(-qkac*dxy, -qkac*dy*dy, -kas*dy);
    N += vec3(dx*kas, dy*kas, -qkac);
  }

  // clamp vertical displacement relative to original vertex.z to avoid extreme peaks
  float baseZ = vertex.z;
  float minZ = baseZ - max_disp;
  float maxZ = baseZ + max_disp;
  P.z = clamp(P.z, minZ, maxZ);

  B = normalize(B)*rescale;
  T = normalize(T)*rescale;
  N = normalize(N);
  outVs.rotMatrix = mat3(B, T, N);

  gl_Position = worldviewproj_matrix * P;

  outVs.bumpCoord = uv0.xy*bumpScale + t*bumpSpeed;

  outVs.eyeVec = P.xyz - camera_position_object_space;
  outVs.height = totalHeight;
}
