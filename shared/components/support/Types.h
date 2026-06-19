#ifndef TYPES_H
#define TYPES_H

#if defined(__GNUC__)
#include <stdint.h>
#else
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
#endif

typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#ifndef ARBATOS_PACKED_STRUCT
#if defined(__GNUC__) || defined(__clang__)
#define ARBATOS_PACKED_STRUCT struct __attribute__((packed))
#elif defined(__CC_ARM) || defined(__ICCARM__)
#define ARBATOS_PACKED_STRUCT __packed struct
#else
#define ARBATOS_PACKED_STRUCT struct
#endif
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

#endif
