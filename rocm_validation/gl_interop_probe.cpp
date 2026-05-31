// Headless EGL + HIP GL-interop feasibility probe for ElasticFusion on gfx90a.
// Creates a hardware (radeonsi) GL context on an amdgpu DRM render node via the
// EGL device platform, makes a GL_TEXTURE_2D (RGBA32F) with a known pattern, then
// registers it with hipGraphicsGLRegisterImage, maps it, gets the hipArray, and
// copies it back with hipMemcpy2DFromArray to verify the data round-trips to a
// HIP-readable array. This is the gate that decides whether ElasticFusion's
// GL->CUDA tracking ingest can use real interop on this headless node.

#define EGL_EGLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <GL/glext.h>

#define __HIP_PLATFORM_AMD__
#include <hip/hip_runtime.h>
#include <hip/hip_gl_interop.h>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

#ifndef GL_RGBA32F
#define GL_RGBA32F 0x8814
#endif

static const char* eglErrStr(EGLint e) {
  switch (e) {
    case EGL_SUCCESS: return "EGL_SUCCESS";
    case EGL_BAD_DISPLAY: return "EGL_BAD_DISPLAY";
    case EGL_NOT_INITIALIZED: return "EGL_NOT_INITIALIZED";
    case EGL_BAD_ACCESS: return "EGL_BAD_ACCESS";
    case EGL_BAD_ALLOC: return "EGL_BAD_ALLOC";
    case EGL_BAD_ATTRIBUTE: return "EGL_BAD_ATTRIBUTE";
    case EGL_BAD_CONFIG: return "EGL_BAD_CONFIG";
    case EGL_BAD_CONTEXT: return "EGL_BAD_CONTEXT";
    case EGL_BAD_MATCH: return "EGL_BAD_MATCH";
    case EGL_BAD_PARAMETER: return "EGL_BAD_PARAMETER";
    default: return "EGL_?";
  }
}

#define HIPCHECK(x) do { hipError_t e=(x); if(e!=hipSuccess){ \
  printf("HIP FAIL %s -> %s (%s)\n", #x, hipGetErrorName(e), hipGetErrorString(e)); } } while(0)

int main(int argc, char** argv) {
  const int W = 64, H = 48;

  // --- 1. Bring up an EGL context on an amdgpu DRM render node (no X server). ---
  PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
      (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
  PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
      (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
  PFNEGLQUERYDEVICESTRINGEXTPROC eglQueryDeviceStringEXT =
      (PFNEGLQUERYDEVICESTRINGEXTPROC)eglGetProcAddress("eglQueryDeviceStringEXT");

  EGLDisplay dpy = EGL_NO_DISPLAY;

  if (eglQueryDevicesEXT && eglGetPlatformDisplayEXT) {
    EGLDeviceEXT devs[16];
    EGLint ndev = 0;
    eglQueryDevicesEXT(16, devs, &ndev);
    printf("EGL devices: %d\n", ndev);
    int want = (argc > 1) ? atoi(argv[1]) : -1;  // optional device index
    for (int i = 0; i < ndev; ++i) {
      const char* drm = eglQueryDeviceStringEXT ?
          eglQueryDeviceStringEXT(devs[i], EGL_DRM_DEVICE_FILE_EXT) : nullptr;
      const char* vend = eglQueryDeviceStringEXT ?
          eglQueryDeviceStringEXT(devs[i], EGL_EXTENSIONS) : nullptr;
      printf("  dev[%d] drm=%s ext=%.60s\n", i, drm ? drm : "(none)", vend ? vend : "");
    }
    for (int i = 0; i < ndev; ++i) {
      if (want >= 0 && i != want) continue;
      const char* drm = eglQueryDeviceStringEXT ?
          eglQueryDeviceStringEXT(devs[i], EGL_DRM_DEVICE_FILE_EXT) : nullptr;
      // skip software devices (no DRM file)
      if (want < 0 && !drm) continue;
      EGLDisplay d = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, devs[i], nullptr);
      if (d != EGL_NO_DISPLAY) { dpy = d; printf("using EGL dev[%d] drm=%s\n", i, drm?drm:"(none)"); break; }
    }
  }
  if (dpy == EGL_NO_DISPLAY) {
    printf("falling back to EGL_DEFAULT_DISPLAY (surfaceless)\n");
    dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  }
  if (dpy == EGL_NO_DISPLAY) { printf("no EGL display\n"); return 2; }

  EGLint major, minor;
  if (!eglInitialize(dpy, &major, &minor)) {
    printf("eglInitialize failed: %s\n", eglErrStr(eglGetError())); return 2;
  }
  printf("EGL %d.%d vendor=%s\n", major, minor, eglQueryString(dpy, EGL_VENDOR));

  const EGLint cfgAttr[] = {
    EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
    EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
    EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
    EGL_NONE
  };
  EGLConfig cfg; EGLint ncfg = 0;
  if (!eglChooseConfig(dpy, cfgAttr, &cfg, 1, &ncfg) || ncfg < 1) {
    printf("eglChooseConfig failed: %s\n", eglErrStr(eglGetError())); return 2;
  }
  if (!eglBindAPI(EGL_OPENGL_API)) { printf("eglBindAPI(GL) failed: %s\n", eglErrStr(eglGetError())); }
  // Try a few context attribute sets; Mesa device-platform can be picky about profile/version.
  static const EGLint a0[] = { EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 6,
      EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT, EGL_NONE };
  static const EGLint a1[] = { EGL_CONTEXT_MAJOR_VERSION, 3, EGL_CONTEXT_MINOR_VERSION, 3,
      EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL_NONE };
  static const EGLint a2[] = { EGL_CONTEXT_MAJOR_VERSION, 3, EGL_CONTEXT_MINOR_VERSION, 0, EGL_NONE };
  static const EGLint a3[] = { EGL_NONE };
  const EGLint* tries[] = { a0, a1, a2, a3 };
  EGLContext ctx = EGL_NO_CONTEXT;
  for (int t = 0; t < 4 && ctx == EGL_NO_CONTEXT; ++t) {
    ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, tries[t]);
    if (ctx == EGL_NO_CONTEXT)
      printf("eglCreateContext try %d failed: %s\n", t, eglErrStr(eglGetError()));
    else
      printf("eglCreateContext try %d OK\n", t);
  }
  if (ctx == EGL_NO_CONTEXT) { printf("RESULT: NO_GL_CONTEXT\n"); return 2; }
  if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
    printf("eglMakeCurrent(surfaceless) failed: %s\n", eglErrStr(eglGetError())); return 2;
  }
  printf("GL_RENDERER=%s\n", (const char*)glGetString(GL_RENDERER));
  printf("GL_VERSION=%s\n", (const char*)glGetString(GL_VERSION));

  // --- 2. Which HIP device does this GL context map to? ---
  unsigned int hipCount = 0;
  int hipDevs[8] = {0};
  hipError_t glg = hipGLGetDevices(&hipCount, hipDevs, 8, hipGLDeviceListAll);
  printf("hipGLGetDevices -> %s, count=%u", hipGetErrorName(glg), hipCount);
  for (unsigned i = 0; i < hipCount && i < 8; ++i) printf(" dev[%d]=%d", i, hipDevs[i]);
  printf("\n");

  int hipDev = -1;
  HIPCHECK(hipGetDevice(&hipDev));
  hipDeviceProp_t prop; HIPCHECK(hipGetDeviceProperties(&prop, hipDev < 0 ? 0 : hipDev));
  printf("current HIP device=%d name=%s gcnArch=%s\n", hipDev, prop.name, prop.gcnArchName);

  // --- 3. Make a GL_TEXTURE_2D RGBA32F with a known pattern. ---
  std::vector<float> pattern(W * H * 4);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      float* p = &pattern[(y * W + x) * 4];
      p[0] = (float)x; p[1] = (float)y; p[2] = (float)(x * y); p[3] = 1.0f;
    }
  GLuint tex = 0;
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, W, H, 0, GL_RGBA, GL_FLOAT, pattern.data());
  glFinish();
  GLenum gle = glGetError();
  printf("glTexImage2D RGBA32F %dx%d glErr=0x%x tid=%u\n", W, H, gle, tex);

  // --- 4. Register + map + get array, exactly as GPUTexture/RGBDOdometry do. ---
  hipGraphicsResource* res = nullptr;
  hipError_t reg = hipGraphicsGLRegisterImage(&res, tex, GL_TEXTURE_2D, hipGraphicsRegisterFlagsReadOnly);
  printf("hipGraphicsGLRegisterImage -> %s (%s)\n", hipGetErrorName(reg), hipGetErrorString(reg));
  if (reg != hipSuccess) { printf("RESULT: INTEROP_REGISTER_FAILED\n"); return 3; }

  HIPCHECK(hipGraphicsMapResources(1, &res, 0));
  hipArray_t arr = nullptr;
  hipError_t ga = hipGraphicsSubResourceGetMappedArray(&arr, res, 0, 0);
  printf("hipGraphicsSubResourceGetMappedArray -> %s arr=%p\n", hipGetErrorName(ga), (void*)arr);
  if (ga != hipSuccess || !arr) { printf("RESULT: INTEROP_GETARRAY_FAILED\n"); return 4; }

  // --- 5. Copy array back to host and verify the round-trip. ---
  std::vector<float> got(W * H * 4, -123.0f);
  hipError_t cp = hipMemcpy2DFromArray(got.data(), W * 4 * sizeof(float), arr, 0, 0,
                                       W * 4 * sizeof(float), H, hipMemcpyDeviceToHost);
  printf("hipMemcpy2DFromArray(D2H) -> %s\n", hipGetErrorName(cp));

  // Also test the D2D path (what initICP uses) into a hipMalloc'd buffer then back.
  float* dbuf = nullptr;
  HIPCHECK(hipMalloc(&dbuf, W * H * 4 * sizeof(float)));
  hipError_t cp2 = hipMemcpy2DFromArray(dbuf, W * 4 * sizeof(float), arr, 0, 0,
                                        W * 4 * sizeof(float), H, hipMemcpyDeviceToDevice);
  printf("hipMemcpy2DFromArray(D2D) -> %s\n", hipGetErrorName(cp2));

  HIPCHECK(hipGraphicsUnmapResources(1, &res, 0));
  HIPCHECK(hipGraphicsUnregisterResource(res));

  if (cp != hipSuccess) { printf("RESULT: INTEROP_COPYBACK_FAILED\n"); return 5; }

  // Verify
  int mism = 0; int firstx=-1, firsty=-1;
  for (int y = 0; y < H && mism < 8; ++y)
    for (int x = 0; x < W && mism < 8; ++x) {
      const float* a = &pattern[(y * W + x) * 4];
      const float* b = &got[(y * W + x) * 4];
      for (int c = 0; c < 4; ++c) {
        if (a[c] != b[c]) {
          if (mism == 0) { firstx = x; firsty = y; }
          if (mism < 8) printf("  mismatch (%d,%d)c%d exp %.1f got %.1f\n", x, y, c, a[c], b[c]);
          ++mism;
        }
      }
    }
  printf("sample got[0..3]=%.1f %.1f %.1f %.1f  got[(1,0)]=%.1f %.1f\n",
         got[0], got[1], got[2], got[3], got[4], got[5]);
  if (mism == 0) {
    printf("RESULT: INTEROP_OK (texture round-trips to HIP array, D2D=%s)\n",
           hipGetErrorName(cp2));
    return 0;
  } else {
    printf("RESULT: INTEROP_DATA_MISMATCH first=(%d,%d) count>=%d\n", firstx, firsty, mism);
    return 6;
  }
}
