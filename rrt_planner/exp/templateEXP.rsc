# TEMPLATE for experience
space-separated values, one topic per line
-- begin template --
title
D x y w h 	// position of the space width & height
S x y 		// start point coords x&y
G x y w h 	// goal region (x,y) (lower left point) width & height
O x y w h	// obstacle x,y,w,h (same rule) //MULTIPLE
-- end template --

-- begin example --
basicExp
D 20 20
S 0 0
G 10 10 1 1
O 4 4 1 1
O 15 12 2 1
-- end example --
