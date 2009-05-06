#!/usr/bin/env octave

% Really simple script for building the command file for doing the CB tests

lift_min   = -.5 + .17 ;
lift_max   = 1.3 - .05 ;
lift_range = lift_max-lift_min ;
lift_num   = 20 ;
lift_pos = linspace(lift_min, lift_max, lift_num) ;

flex_min   = -2.3 + .25 ;
flex_max   =  0.1 - .1 ;
flex_range = flex_max - flex_min ;
flex_num   = 8 ;
flex_pos   = linspace(flex_min, flex_max, flex_num) ;

[lift_pos_arr, flex_pos_arr] = meshgrid(lift_pos, flex_pos) ;

fid = fopen('joint_pos.txt', 'w') ;
if (fid<0)
    error('Error opening file') ;
end

for k=1:length(lift_pos_arr(:))
    fprintf(fid, '0.00 % .5f 0.0 % .5f 0.00 0.00 0.00\n', ...
                 lift_pos_arr(k), flex_pos_arr(k)) ;
end
fclose(fid) ;