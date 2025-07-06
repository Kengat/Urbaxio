#ifndef URBAXIO_ENGINE_H
#define URBAXIO_ENGINE_H

#include <cstdint>

#ifdef _WIN32
#ifdef URBAXIO_ENGINE_BUILD_SHARED
#define URBAXIO_API __declspec(dllexport)
#else
#define URBAXIO_API // __declspec(dllimport)
#endif
#else // Linux, macOS
#define URBAXIO_API __attribute__((visibility("default")))
#endif

//                                 Scene     C API
typedef struct Scene Scene;

#ifdef __cplusplus
extern "C" {
#endif

    URBAXIO_API void initialize_engine();
    URBAXIO_API Scene* get_engine_scene(); //                                  

    // TODO:          API                       (        /                     . .)

#ifdef __cplusplus
} // extern "C"
#endif

#endif // URBAXIO_ENGINE_H