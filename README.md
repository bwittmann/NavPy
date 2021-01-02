# Trash_Picking_Robot

TODO: please enter your created packages below

## Packages

### rto_map_server
This package includes a node called 'map_server'.

The map server transforms a .pgm file from the maps folder and adds meta information which is stored in the corresponding .yaml file. 
This map server also works with multiple maps.
To add a map to the map server simply append the map_server_params.yaml file in the config folder of the rto_map_server package.
Keep in mind that the syntax has to match the syntax of 'map1' and the 'maps_nr' has to be updated.

The input to the map server matches the commonly used structure from the ROS navigation stack.

To launch the map server have a look at the launch folder of this package.

#### Service 'get_map'
request: map number (int64)  
response: map (OccupancyGrid)

### rto_costmap_generator (under construction)
This package includes a node called 'costmap_generator'.

The costmap generator creates a global costmap that is based on the static map from the map server and has been padded in order to allow the use of a point 
representation of the mobile robot for path planning. There exist two types of padding: hard (val: 100) and soft padding (val: < 100). Hard padded cells should under no circumstances be  visited by the robot while soft padded cells can be visited by the robot. Soft padded cells increase the cost that is estimated by the planning algorithm. 

To launch the costmap generator have a look at the launch folder of this package.

In the future the costmap generator should also be able to change the global costmap based on newly observed obstacles in the dynamic environment.

### Service 'switch_maps'
request: map_nr_switch (int8)
response: sucess (bool)

### rto_global_planner
This package includes a node called 'rto_global_planner'.

The main function is to generate a global path. It is now according to costmap provided by move_base node, still some problems with our own costmap and map server. 

To see the global path in rviz, run the following code in terminal.

        roslaunch rto_global_planner navigation.launch
        
        roslaunch rto_global_planner global_planner.launch


## Simulation Worlds

To use the additional simulation worlds please clone the following repo:

        https://github.com/chaolmu/gazebo_models_worlds_collection

Furthermore add the following line to your .bashrc file:

        export GAZEBO_MODEL_PATH=<path to repo>/gazebo_models_worlds_collection/models

Keep in mind that the name of the used map has to match the name of the corresponding world (eg. sim_simple)

