#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__
#include <stdint.h>

typedef unsigned short uint16;
typedef unsigned int uint32;
#if defined _MSC_VER || defined __BORLANDC__
    typedef __int64 int64;
    typedef unsigned __int64 uint64;
#else
    #include <inttypes.h>
/***
 * REF:
 *     https://github.com/goldendict/goldendict/issues/714
 *     http://blog.csdn.net/shiwei408/article/details/7463476
 *
 * NOTICE:
 * qendian.h have qbswap template for quint64 (defined as "unsigned long long" in qglobal.h) parameter
 * but haven't it for uint64_t (defined as "unsigned long" on UNIX-like systems).
*/
//    typedef /*int64_t*/long long int64;   // conflicting declaration with opencv!!!
//    typedef /*uint64_t*/unsigned long long uint64;
    typedef int64_t/*long long*/ int64;   // conflicting declaration with opencv!!!
    typedef uint64_t/*unsigned long long*/ uint64;
#endif

typedef unsigned char uchar;
typedef unsigned char byte;
typedef unsigned char uint8;
typedef char int8;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

#ifdef _WIN32
    #include <windows.h>
#else
typedef void *LPVOID;
typedef unsigned long       DWORD;
typedef bool                BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned int        UINT;
typedef char            CHAR, TCHAR;
typedef unsigned short  USHORT;
typedef unsigned int    UINT32;
typedef int             INT32;
#endif


#ifdef TRUE
#undef TRUE
#endif
#define TRUE true

#ifdef FALSE
#undef FALSE
#endif
#define FALSE false

struct T_Rect
{
    int X; // from 0
    int Y;
    int Width;
    int Height;
};

#endif // __CZCMTIDEF_H__
