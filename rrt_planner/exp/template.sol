###
# Solution file format
###
# space-separated values, one topic per line
#
-- begin template --
title 					// string		: title of this solution
experienceFileName 		// string		: related file describing the experience
G x y					// char float*2	: goal position actually reached
W x1 y1					// char float*4	: path segment to reach the goal	//MULTIPLE
...
T x1 y1 x2 y2			//char float*4	: segment of the RRT tree			//MULTIPLE
...
-- end template --