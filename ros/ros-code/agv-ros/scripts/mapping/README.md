# start_gmapper.bash
This starts gmapper. This will start the process of creating a new map.
# start_localizer.bash
This will read in a map at src/myagv_navigation2/map/ and localize the robot withini it.
# save_map.bash
This will save the map that gmapper (or whatever is publishing to the map topic) to your current directory as "new_map.pgm" and "new_map.yaml".

You need to rename these maps to "map.pgm" and "map.yaml" and edit "map.yaml" to refer to "map.pgm" instead of "new_map.pgm".
These two files must be placed under src/myagv_navigation2/map/.
