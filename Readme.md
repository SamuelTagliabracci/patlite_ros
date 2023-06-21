#Launch
roslaunch patlite_ros patlite.launch

#Example Run
rostopic pub -1 /patlite/color "data: 4"

#/patlite/color
0 - OFF
1 - RED
2 - GREEN
3 - YELLOW
4 - BLUE
5 - PURPLE
6 - LIGHTBLUE
7 - WHITE

#/patlite/color_pattern
0 - OFF
1 - SOLID
2 - FLASHING
3 - SLOW FLASHING
4 - DOUBLE FLASH + PAUSE
5 - QUICK FLASH
6 - SINGLE FLASH + PAUSE
7 - AURA GLOW

#/patlite/buzzer
0 - BUZZER_OFF
1 - BUZZER_ON
2 - Blow (continuous)
3 - Sweep sound
4 - Intermittent sound"""
4 - WEAK_ATTENTION
5 - STRONG_ATTENTION
6 - TWINKLE_TWINKLE
7 - LONDON_BRIDGE

#/patlite/buzzer_volume/
0 - OFF
1 - 10 VOLUME
