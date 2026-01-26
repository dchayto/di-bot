/*								screen_handler.hpp							//
	helper functions to handle screen output for keyboard handler interface

	keeping intentionally simple, and just displaying project name, commands,
	and current gain. everything on screen will be statically written, aside
	from gain, which will be overwritten whenever gain is changed.

	may expand this in the future to allow for display of robot state, when
	more than one divisible state is present (currently just "manual driving"
	state, but there may be some utility in adding 

	this may also be expanded/rewritten in the future to display status
	messages (critical errors, etc.), but currently i'm thinking to have that
	info be in a separate terminal window (i.e., opening a window to display
	all RCLCPP info/warning messages and tiling with other windows)
	  -> LOOK INTO TMUX FOR TILING - see if this will work over ssh (should)
*/
#ifndef __KB_SCREEN_HANDLER__
#define __KB_SCREEN_HANDLER__

#include <iostream>
#include <iomanip>
#include <cstdlib> 	// for std::system("clear")

#include "kb_input.hpp"

namespace ScreenHandler	{
	void wipe()	{
		std::system("clear");
	}

	void writeGain(uint8_t gain)	{
		// NOTE: this function assumes gain is last value written to screen,
		// and cursor is on final digit of gain value (expressing as 3 chars)
		std::cout << "\b\b\b"; 	// move cursor back 3 chars
		std::cout << std::setw(3) << gain;
		std::cout << std::flush;
	}

	void initScreen()	{
		wipe();
		// ALSO HANDLE SCREEN FORMATTING - look into this
		
		// write string with project name - looks a little fucky b/c of escape chars
		std::cout << 	
"\n                                                                           "
"\n             ____  ________  _______        __ ___       ______ ___        "
"\n            / __ \\/ ____/  |/  / __ \\      / // / |     / / __ <  /        "
"\n           / / / / /   / /|_/ / /_/ /_____/ // /| | /| / / / / / /         " 
"\n          / /_/ / /___/ /  / / _, _/_____/__  __/ |/ |/ / /_/ / /          " 
"\n         /_____/\\____/_/  /_/_/ |_|        /_/  |__/|__/\\____/_/           " 
"\n                                                                           " 
"\n                                                                           "
		; // end of project title string
		
		// write string for instructions
		std::cout << 	
			"				\n"
			"            	\n"
			"				\n"
			"				\n"
		; // end of instruction string
		
		// write string for gain (+ 3 blank spaces for gain value)
		std::cout << "Gain:    ";
		std::cout << std::flush;
	}

}

#endif
