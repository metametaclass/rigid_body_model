# Now plot the data with lines and points
plot 'result.txt' using 1:2 w lp title 'y1', \
     '' using 1:3 w lp title 'y2', \
     '' using 1:4 w lp title 'y3', \
     '' using 1:5 w lp title 'y4'

set terminal wxt 1
plot 'result.txt' using 2:3 w lp title 'y1/y2'

set terminal wxt 2
plot 'result.txt' using 4:5 w lp title 'y3/y4'

set terminal wxt 4
plot 'result.txt' using 2:4 w lp title 'y1/y3'

set terminal wxt 5
plot 'result.txt' using 3:5 w lp title 'y2/y4'

pause mouse close
