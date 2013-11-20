#ifndef __AVRDEF_H
#define __AVRDEF_H


#ifndef BIT
#define BIT(x)	_BV(x)
#endif


#define SEI()	sei()
#define CLI()	cli()


// #define PROGMEM __attribute__((__progmem__))


/*
 
#define _asm	asm			// old style

#define SEI()	asm("sei")
#define CLI()	asm("cli")

#define WDR()	asm("wdr")
#define NOP()	asm("nop")
#define SLEEP() asm("sleep");
#define _WDR()	asm("wdr")
#define _SEI()	asm("sei")
#define _CLI()	asm("cli")
#define _NOP()	asm("nop")
#define _SLEEP() asm("sleep");

*/

#endif
