

#ifndef _PRINTF_H_
#define _PRINTF_H_

#include "cpu.h"

#include "error.h"
#include "uart.h"


#ifndef NULL
 #define NULL 0
#endif


#if defined(va_start) && defined(va_arg) && defined(va_end)
  typedef struct __va_list va_list;
#else
  typedef int va_list;
  #define _INTSIZEOF(n)  ((sizeof(n)+sizeof(int)-1)&~(sizeof(int) - 1) )
  #define va_start(ap,v) ( ap = (va_list)&v + _INTSIZEOF(v) )
  #define va_arg(ap,t)   ( *(t *)((ap += _INTSIZEOF(t)) - _INTSIZEOF(t)) ) 
  #define va_end(ap)     ( ap = (va_list)0 )
#endif


int print(const char *, ...);

#endif

