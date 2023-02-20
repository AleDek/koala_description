# koala_description

package con urdf del drone per avere le tf.
il launch file koala_pipe_sim_.launch provvede a lanciare il sitl in gazebo, qground se ci metti il path giusto e il robot_state_publisher con rviz per monitorare la situazione.

per lanciare il sitl con i wrapper ros bisognaaggiungere al .bashrc le seguenti righe, ipotizzando di aver clonato px4 in ~/px4_devel/PX4-Autopilot 

### PX4 package for ros MIO SIM TESI
source ~/px4_devel/PX4-Autopilot/Tools/setup_gazebo.bash ~/px4_devel/PX4-Autopilot ~/px4_devel/PX4-Autopilot/build/px4_sitl_default >/dev/null
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/px4_devel/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/px4_devel/PX4-Autopilot/Tools/sitl_gazebo
