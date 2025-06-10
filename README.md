# wheel_legged
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build ocs2_self_collision_visualization
catkin build wheel_legged_controllers wheel_legged_description wheel_legged_mujoco mujoco lcm_msg
catkin build wheel_legged_gazebo
catkin build wheel_legged_controllers wheel_legged_description mujoco_sim

roslaunch wheel_legged_description empty_world.launch

roslaunch wheel_legged_controllers load_controller.launch cheater:=false

rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/wheel_legged_controller']                   
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0" 

这个版本相对来说是最完整的，完整的完成了MPC-WBC对于轮腿的仿真部署，接下来就是实机代码
