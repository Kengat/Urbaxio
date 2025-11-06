#pragma once

#ifdef _WIN32
  #ifdef URBAXIO_ENGINE_BUILD_SHARED
    #define URBAXIO_ENGINE_API __declspec(dllexport)
  #else
    #define URBAXIO_ENGINE_API __declspec(dllimport)
  #endif
#else // Linux, macOS
  #define URBAXIO_ENGINE_API __attribute__((visibility("default")))
#endif

