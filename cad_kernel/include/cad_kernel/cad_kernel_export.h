#pragma once

#ifdef _WIN32
  #ifdef URBAXIO_CAD_KERNEL_BUILD_SHARED
    #define URBAXIO_CAD_API __declspec(dllexport)
  #else
    #define URBAXIO_CAD_API __declspec(dllimport)
  #endif
#else // Linux, macOS
  #define URBAXIO_CAD_API __attribute__((visibility("default")))
#endif

