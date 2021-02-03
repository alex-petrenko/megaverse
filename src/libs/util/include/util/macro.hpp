#pragma once


// compiler-dependent attributes
#if defined(__clang__) || defined(__GNUG__)
    #define FORCE_INLINE inline __attribute__((always_inline))
#else
    #define FORCE_INLINE __forceinline
#endif


#define EPSILON 1e-5f


#define ARR_LENGTH(arr) sizeof(arr) / sizeof(arr[0])

#define UNUSED(x) (void)(x)
