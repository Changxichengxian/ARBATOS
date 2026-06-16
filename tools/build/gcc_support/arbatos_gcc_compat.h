#ifndef ARBATOS_GCC_COMPAT_H
#define ARBATOS_GCC_COMPAT_H

#if defined(__GNUC__) && !defined(__CC_ARM)
#ifndef __weak
#define __weak __attribute__((weak))
#endif

#ifndef __packed
#define __packed __attribute__((packed))
#endif

#ifndef __align
#define __align(x) __attribute__((aligned(x)))
#endif

#ifndef __inline
#define __inline inline
#endif
#endif

#endif
