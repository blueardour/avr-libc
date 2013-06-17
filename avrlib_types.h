
//*****************************************************************************


#ifndef _AVRLIBTYPES_H_
#define _AVRLIBTYPES_H_

// datatype definitions macros
typedef unsigned char  u08;
typedef unsigned char  u8;
typedef   signed char  s08;
typedef   signed char  s8;
typedef unsigned short u16;
typedef   signed short s16;
typedef unsigned long  u32;
typedef   signed long  s32;
typedef unsigned long long u64;
typedef   signed long long s64;


// maximum value that can be held
// by unsigned data types (8,16,32bits)
#define MAX_U08	255
#define MAX_U16	65535
#define MAX_U32	4294967295

// maximum values that can be held
// by signed data types (8,16,32bits)
#define MIN_S08	-128
#define MAX_S08	127
#define MIN_S16	-32768
#define MAX_S16	32767
#define MIN_S32	-2147483648
#define MAX_S32	2147483647

#define FALSE 0
#define TRUE  1

#ifndef __WIN32__
typedef unsigned char   BOOL;
typedef unsigned char	BYTE;
typedef unsigned int	WORD;
typedef unsigned long	DWORD;
typedef unsigned char	UCHAR;
typedef unsigned char	uchar;
typedef unsigned int	UINT;
typedef unsigned short  USHORT;
typedef unsigned long	ULONG;
typedef char			CHAR;
typedef int				INT;
typedef long			LONG;
#endif


#ifdef __ICC_VERSION

#endif

#endif
