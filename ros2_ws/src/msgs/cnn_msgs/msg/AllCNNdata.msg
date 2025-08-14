# ROBOT
geometry_msgs/PoseStamped robot_pose
float32[] vel                 # current velocity in robot frame
float32[] scan	  	# 720 range data from the laser scan
float32[] scan_all	# 1080 range data from the laser scan

# GOAL
nav_msgs/Path global_path
geometry_msgs/PoseStamped global_goal
geometry_msgs/Point     local_goal_from_map
geometry_msgs/Point     local_goal_from_robot
float32[] goal_cart     # current goal in robot frame
float32[] goal_final_polar    # final goal in robot frame

# AGENTS
agents_msgs/AgentArray agents
float32[] ped_pos_map		# pedestrian's positon costmap in Cartesian coordinate: 2 channel, 20m * 20m

# IMAGE
float32[] image_gray	        # image data from the zed camera
float32[] depth	        # depth image data from the zed camera