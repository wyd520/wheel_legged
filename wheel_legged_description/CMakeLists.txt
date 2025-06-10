cmake_minimum_required(VERSION 3.10)
project(wheel_legged_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roslaunch
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
)

#############
## Install ##
#############

# Mark resource files for installation
foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
