//
// portable_endian.h
//
// https://gist.github.com/panzi/6856583
//
// I, Mathias Panzenböck, place this file hereby into the public domain. Use
// it at your own risk for whatever you like. In case there are
// jurisdictions that don't support putting things in the public domain you
// can also consider it to be "dual licensed" under the BSD, MIT and Apache
// licenses, if you want to. This code is trivial anyway. Consider it an
// example on how to get the endian conversion functions on different
// platforms.

#ifndef UR_CLIENT_LIBRARY_PORTABLE_ENDIAN_H_INCLUDED
#define UR_CLIENT_LIBRARY_PORTABLE_ENDIAN_H_INCLUDED

// Byte order
#if defined(linux) || defined(__linux) || defined(__linux__) || defined(__CYGWIN__)
#  include <endian.h>
#elif defined(__APPLE__)
#  include <libkern/OSByteOrder.h>

#  ifndef htobe16
#    define htobe16(x) OSSwapHostToBigInt16(x)
#  endif
#  ifndef htole16
#    define htole16(x) OSSwapHostToLittleInt16(x)
#  endif
#  ifndef be16toh
#    define be16toh(x) OSSwapBigToHostInt16(x)
#  endif
#  ifndef le16toh
#    define le16toh(x) OSSwapLittleToHostInt16(x)
#  endif

#  ifndef htobe32
#    define htobe32(x) OSSwapHostToBigInt32(x)
#  endif
#  ifndef htole32
#    define htole32(x) OSSwapHostToLittleInt32(x)
#  endif
#  ifndef be32toh
#    define be32toh(x) OSSwapBigToHostInt32(x)
#  endif
#  ifndef le32toh
#    define le32toh(x) OSSwapLittleToHostInt32(x)
#  endif

#  ifndef htobe64
#    define htobe64(x) OSSwapHostToBigInt64(x)
#  endif
#  ifndef htole64
#    define htole64(x) OSSwapHostToLittleInt64(x)
#  endif
#  ifndef be64toh
#    define be64toh(x) OSSwapBigToHostInt64(x)
#  endif
#  ifndef le64toh
#    define le64toh(x) OSSwapLittleToHostInt64(x)
#  endif

#  ifndef __BYTE_ORDER
#    define __BYTE_ORDER BYTE_ORDER
#  endif
#  ifndef __BIG_ENDIAN
#    define __BIG_ENDIAN BIG_ENDIAN
#  endif
#  ifndef __LITTLE_ENDIAN
#    define __LITTLE_ENDIAN LITTLE_ENDIAN
#  endif
#  ifndef __PDP_ENDIAN
#    define __PDP_ENDIAN PDP_ENDIAN
#  endif
#elif defined(__OpenBSD__)
#  include <sys/endian.h>
#elif defined(__NetBSD__) || defined(__FreeBSD__) || defined(__DragonFly__)
#  include <sys/endian.h>

#  ifndef be16toh
#    define be16toh(x) betoh16(x)
#  endif
#  ifndef le16toh
#    define le16toh(x) letoh16(x)
#  endif

#  ifndef be32toh
#    define be32toh(x) betoh32(x)
#  endif
#  ifndef le32toh
#    define le32toh(x) letoh32(x)
#  endif

#  ifndef be64toh
#    define be64toh(x) betoh64(x)
#  endif
#  ifndef le64toh
#    define le64toh(x) letoh64(x)
#  endif
#elif defined(_WIN32)
#  include <stdlib.h>
#  if BYTE_ORDER == LITTLE_ENDIAN
#    if defined(_MSC_VER)
#      ifndef htobe16
#        define htobe16(x) _byteswap_ushort(x)
#      endif
#      ifndef htole16
#        define htole16(x) (x)
#      endif
#      ifndef be16toh
#        define be16toh(x) _byteswap_ushort(x)
#      endif
#      ifndef le16toh
#        define le16toh(x) (x)
#      endif

#      ifndef htobe32
#        define htobe32(x) _byteswap_ulong(x)
#      endif
#      ifndef htole32
#        define htole32(x) (x)
#      endif
#      ifndef be32toh
#        define be32toh(x) _byteswap_ulong(x)
#      endif
#      ifndef le32toh
#        define le32toh(x) (x)
#      endif

#      ifndef htobe64
#        define htobe64(x) _byteswap_uint64(x)
#      endif
#      ifndef htole64
#        define htole64(x) (x)
#      endif
#      ifndef be64toh
#        define be64toh(x) _byteswap_uint64(x)
#      endif
#      ifndef le64toh
#        define le64toh(x) (x)
#      endif
#    elif defined(__GNUC__) || defined(__clang__)
#      ifndef htobe16
#        define htobe16(x) __builtin_bswap16(x)
#      endif
#      ifndef htole16
#        define htole16(x) (x)
#      endif
#      ifndef be16toh
#        define be16toh(x) __builtin_bswap16(x)
#      endif
#      ifndef le16toh
#        define le16toh(x) (x)
#      endif

#      ifndef htobe32
#        define htobe32(x) __builtin_bswap32(x)
#      endif
#      ifndef htole32
#        define htole32(x) (x)
#      endif
#      ifndef be32toh
#        define be32toh(x) __builtin_bswap32(x)
#      endif
#      ifndef le32toh
#        define le32toh(x) (x)
#      endif

#      ifndef htobe64
#        define htobe64(x) __builtin_bswap64(x)
#      endif
#      ifndef htole64
#        define htole64(x) (x)
#      endif
#      ifndef be64toh
#        define be64toh(x) __builtin_bswap64(x)
#      endif
#      ifndef le64toh
#        define le64toh(x) (x)
#      endif
#    else
#      error Compiler is not supported
#    endif
#  else
#    error Byte order is not supported
#  endif

#  ifndef __BYTE_ORDER
#    define __BYTE_ORDER BYTE_ORDER
#  endif
#  ifndef __BIG_ENDIAN
#    define __BIG_ENDIAN BIG_ENDIAN
#  endif
#  ifndef __LITTLE_ENDIAN
#    define __LITTLE_ENDIAN LITTLE_ENDIAN
#  endif
#  ifndef __PDP_ENDIAN
#    define __PDP_ENDIAN PDP_ENDIAN
#  endif
#else
#  error Platform is not supported
#endif

#endif  // UR_CLIENT_LIBRARY_PORTABLE_ENDIAN_H_INCLUDED
