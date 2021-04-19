# Set the output file type
set terminal png

# Set the output file name
set output 'multiple_plots.png'

# Now plot the data with lines and points
plot 'result.txt' using 1:2 w lp title 'y1', \
     '' using 1:3 w lp title 'y2', \
     '' using 1:4 w lp title 'y3', \
     '' using 1:5 w lp title 'y4'

set output 'state_1.png'

plot 'result.txt' using 2:3 w lp title 'y1/y2'

set output 'state_2.png'

plot 'result.txt' using 4:5 w lp title 'y3/y4'

set output 'quaternion.png'

plot 'result.txt' using 1:5 w lp title 'q.w', \
     '' using 1:6 w lp title 'q.x', \
     '' using 1:7 w lp title 'q.y', \
     '' using 1:8 w lp title 'q.z'
