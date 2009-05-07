#!/usr/bin/env octave

% Really simple script for building the command file for doing the CB tests

lift_min   = -.5 + .17 ;
lift_max   = 1.3 - .05 ;
lift_range = lift_max-lift_min ;
lift_num   = 10 ;
lift_pos = linspace(lift_min, lift_max, lift_num) ;

flex_min   = -2.3 + .25 ;
flex_max   =  0.1 - .1 ;
flex_range = flex_max - flex_min ;
flex_num   = 5 ;
flex_pos   = linspace(flex_min, flex_max, flex_num) ;

[lift_pos_arr, flex_pos_arr] = meshgrid(lift_pos, flex_pos) ;

fid = fopen('joint_pos.txt', 'w') ;
if (fid<0)
    error('Error opening file') ;
end

% Build 'upside down' commands
for cur_lift=lift_pos
  cur_flex_min = cur_lift - pi/2 ;
  cur_flex_max = flex_max ;
  cur_flex_pos = flex_pos(flex_pos <= cur_flex_max & flex_pos >= cur_flex_min) ;
  for cur_flex=cur_flex_pos
    fprintf(fid, '0.00 % .5f -3.14 % .5f 0.00 0.00 0.00\n', ...
                 cur_lift, cur_flex) ;
  end
  %keyboard
end

for k=1:length(lift_pos_arr(:))
    fprintf(fid, '0.00 % .5f 0.0 % .5f 0.00 0.00 0.00\n', ...
                 lift_pos_arr(k), flex_pos_arr(k)) ;
end

fclose(fid) ;