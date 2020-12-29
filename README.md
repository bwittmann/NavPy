# Trash_Picking_Robot

## Availabel Services

### /get_map   
request: map number (int64)  
response: map (OccupancyGridMap)

## Map Server
To add a map to the map server simply append the map_server_params.yaml file in the config folder of the rto_map_server package.
Keep in mind that the syntax has to match the syntax of 'map1' and the 'maps_nr' has to be updated.


## Simulation Worlds

To use the additional simulation worlds please clone the following repo:

        https://github.com/chaolmu/gazebo_models_worlds_collection

Furthermore add the following line to your .bashrc file:

        export GAZEBO_MODEL_PATH=<path to repo>/gazebo_models_worlds_collection/models

Keep in mind that the name of the used map has to match the name of the corresponding world (eg. sim_simple)

## Additional .launch Files

To run navigation without move_base use the navigation_no_mb.launch file. Keep in mind that AMCL and the map server are still running.