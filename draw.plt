# Set the output file type
set terminal png

# Set the output file name
set output 'multiple_plots.png'

# Now plot the data with lines and points
plot 'result.txt' using 1:2 w lp title 'y1', \
     '' using 1:3 w lp title 'y2', \
     '' using 1:4 w lp title 'y3', \
     '' using 1:5 w lp title 'y4'
