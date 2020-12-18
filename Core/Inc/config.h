#ifndef __CONFIG
#define __CONFIG

#define PWM_MAX 4096


//TODO: a few (const) structs or array or something predefining "pretty" LED coulours.
// maybe it's a good idea to scale this to max setpoint.
// certainly it's a bad idea to hardcode values
// Does cortex M0 have a hw divider? Float multiplier? Does my compiler use them?
// Meh, premature optimalisation. Besides, I could just use floats, precalculate the arrays, and then use the calculated onces :)

#if 0
#define LEN_COLOR 33
const int colorset_percentage[LEN_COLOR] = //r,g,b
{
 100,0,0,    //red
 100,30,100, //white-ish purple
 100,0,30,   //magenta/pink purple
 100,60,0,   //yellow
 0,0,100,     //blue
 0,100,0,    //green
 30,0,100,		// Blue Purple-ish white
 100,100,100, // white 
 0,100,100,    //bluegreen
 100,0,100,   // purple
 0,100,60	// greenish Blueish?
 };
#endif

/* change order / sort, make into 9 colors so odd multiple of 3 */
#define LEN_COLOR 27
const int colorset_percentage[LEN_COLOR] = //r,g,b
{
		100,0,0,    //red
		100,60,0,   //yellow
		0,100,0,    //green
		0,100,100,  //bluegreen
		0,0,100,    //blue
		30,0,100,	// Blue Purple-ish white
		100,100,100,//white
		100,0,100,  // purple
		100,0,30,   //magenta/pink purple
};

// How this works:  const array is filled with percentages of SETPOINT (Max).
// then setpoints are calculated as percentage of max and stored in a variable aray
// as start of main, further down the variable array is used
// of course this calculation could have been done at compile time as the values never change (At runtime)...

//Because I want to change color differently on catch and (Maybe?) freefall (or other events), I'll offset one of them a bit but use the same colorset.
//If that does not work out, I make a 2nd colourset. Could even pick a random color from set on events, instead of moving in a predetermined patern?


// TODO: more colours, maybe figure out how not to use RAM, then again, plenty of ram and cpu time avaialable...

#endif

