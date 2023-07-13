unset border
unset tics
set samples 15
set lmargin at screen 0
set rmargin at screen 1
set bmargin at screen 0
set tmargin at screen 1
set xrange[-4:4]
set yrange[-3.59808:3.59808]
set term pngcairo mono enhanced size 240,215
set output 'maze.png'
set multiplot
set arrow from -0.500000,-2.598076 to 0.500000,-2.598076 nohead lc'black' lw 2
set arrow from 0.500000,-2.598076 to 1.500000,-2.598076 nohead lc'black' lw 2
set arrow from -0.500000,-0.866025 to 0.500000,-0.866025 nohead lc'black' lw 2
set arrow from -1.000000,-1.732051 to 0.000000,-1.732051 nohead lc'black' lw 2
set arrow from 0.500000,-0.866025 to 1.000000,-1.732051 nohead lc'black' lw 2
set arrow from 1.500000,-2.598076 to 2.000000,-1.732051 nohead lc'black' lw 2
set arrow from 2.000000,-1.732051 to 2.500000,-0.866025 nohead lc'black' lw 2
set arrow from 2.500000,-0.866025 to 3.000000,-0.000000 nohead lc'black' lw 2
set arrow from 1.000000,-0.000000 to 1.500000,-0.866025 nohead lc'black' lw 2
set arrow from 1.500000,-0.866025 to 2.500000,-0.866025 nohead lc'black' lw 2
set arrow from 3.000000,-0.000000 to 2.500000,0.866025 nohead lc'black' lw 2
set arrow from 2.500000,0.866025 to 2.000000,1.732051 nohead lc'black' lw 2
set arrow from 2.000000,1.732051 to 1.500000,2.598076 nohead lc'black' lw 2
set arrow from 2.000000,-0.000000 to 2.500000,0.866025 nohead lc'black' lw 2
set arrow from 1.500000,0.866025 to 2.500000,0.866025 nohead lc'black' lw 2
set arrow from 1.500000,0.866025 to 1.000000,1.732051 nohead lc'black' lw 2
set arrow from 0.500000,2.598076 to -0.500000,2.598076 nohead lc'black' lw 2
set arrow from -0.500000,2.598076 to -1.500000,2.598076 nohead lc'black' lw 2
set arrow from 0.500000,0.866025 to -0.500000,0.866025 nohead lc'black' lw 2
set arrow from 1.000000,1.732051 to 0.000000,1.732051 nohead lc'black' lw 2
set arrow from -0.000000,0.000000 to -0.500000,0.866025 nohead lc'black' lw 2
set arrow from -1.500000,2.598076 to -2.000000,1.732051 nohead lc'black' lw 2
set arrow from -2.000000,1.732051 to -2.500000,0.866025 nohead lc'black' lw 2
set arrow from -2.500000,0.866025 to -3.000000,0.000000 nohead lc'black' lw 2
set arrow from -0.500000,0.866025 to -1.500000,0.866025 nohead lc'black' lw 2
set arrow from -1.000000,1.732051 to -2.000000,1.732051 nohead lc'black' lw 2
set arrow from -1.500000,0.866025 to -2.000000,0.000000 nohead lc'black' lw 2
set arrow from -1.000000,0.000000 to -2.000000,0.000000 nohead lc'black' lw 2
set arrow from -0.500000,-0.866025 to -1.000000,-1.732051 nohead lc'black' lw 2
set arrow from -3.000000,-0.000000 to -2.500000,-0.866025 nohead lc'black' lw 2
set arrow from -2.500000,-0.866025 to -2.000000,-1.732051 nohead lc'black' lw 2
set arrow from -2.000000,-1.732051 to -1.500000,-2.598076 nohead lc'black' lw 2
set arrow from -1.000000,-0.000000 to -0.500000,-0.866025 nohead lc'black' lw 2
set arrow from -1.500000,-0.866025 to -1.000000,-1.732051 nohead lc'black' lw 2
set arrow from -1.000000,-1.732051 to -2.000000,-1.732051 nohead lc'black' lw 2
plot 1/0 notitle
unset multiplot
set output
