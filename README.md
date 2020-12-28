# Trash_Picking_Robot

## Simulation Worlds

To use the additional simulation worlds please clone the following repo:

        https://github.com/chaolmu/gazebo_models_worlds_collection

Furthermore add the following line to your .bashrc file:

        export GAZEBO_MODEL_PATH=<path to repo>/gazebo_models_worlds_collection/models

Keep in mind that the name of the used map has to match the name of the corresponding world (eg. sim_simple)

## Additional .launch Files

To run navigation without move_base use the navigation_no_mb.launch file. Keep in mind that AMCL and the map server are still running.