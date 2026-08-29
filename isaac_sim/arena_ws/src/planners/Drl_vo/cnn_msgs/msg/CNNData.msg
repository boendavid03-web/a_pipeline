# CNN_data.msg

# Pedestrian position costmap in Cartesian coordinates (2 channels, 20m x 20m)
float32[] ped_pos_map

# Laser scan data: 720 range values from the laser scanner
float32[] scan

# Extended laser scan data: 1080 range values from the laser scanner
float32[] scan_all

# Grayscale image data from the ZED camera
float32[] image_gray

# Depth image data from the ZED camera
float32[] depth

# Current goal in the robot frame (Cartesian coordinates)
float32[] goal_cart

# Final goal in the robot frame (polar coordinates)
float32[] goal_final_polar

# Current velocity in the robot frame
float32[] vel
