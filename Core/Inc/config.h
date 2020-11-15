#ifndef __CONFIG
#define __CONFIG

/* TODO: modify / complete for smallrgbjuggle */

#define PWM_MAX 4096

#define LEN_COLOR 33
const int colorset_percentage[LEN_COLOR] = //r,g,b
{
 100,0,0,    //red
 100,30,100, //white-ish purple
 100,0,30,   //magenta/pink purple
 100,60,0,   //yellow
 0,0,100,     //blue
 0,100,0,    //green
 30,0,100,		// untested... Blue Purple-ish?
 100,100,100, // white 
 0,100,100,    //bluegreen
 100,0,100,   // purple
 0,100,60	// untested... greenish Blueish?
 }; 

#endif

