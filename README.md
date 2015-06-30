# Programming-Challenge

To run the command line application, use the following command:
 astar -s 25 2,1 24,24 < in > out
where 'in' is a list of obstacles and 'out' is the output file.

To generate graphs from the output, use the following command:
 graph -Tpng out --reposition 0 0 1 -S 8 -m 0 --blankout 0 in | display -
where 'out' is the result of running astar and 'in' is the list of
obstacles provided to astar on stdin.

Format of obstacles and output is a list of points. Each point is a pair of
two integers x and y. Each pair is separated by a newline.
Example:
 1 1
 2 3
 5 5

Format of 'start' and 'target' is a pair of points delimited by a comma.
Example:
 2,5
