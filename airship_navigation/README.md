# airship_navigation
**airship_navigation provides a navigation service interface for mobile platforms, allowing them to receive 2D navigation commands and perform path planning and movement functions. This module integrates nav2_simple_commander. By analyzing path planning results and movement status, airship_navigation can provide more detailed navigation status information.**

## airship_navigation API Overview

1. Service
* `/airship_navigation/navigate_to_pose`: Provides a service to execute navigation and return the status.
``` 
# The following example demonstrates sending a coordinate (m) and orientation (rad) to a service and waiting to receive a response.
ros2 service call /airship_navigation/navigate_to_pose airship_interface/srv/AirshipNav "{x: 8.5, y: 8.3, theta: 1.3}"
```

2. Parameters
* `max_speed`: The maxium speed of robot for estimating the time of arrival. (Default: `0.4`)
* `min_runtime`:  The minimum runtime for a single navigation, preventing the robot from timing out during short-distance navigation. (Default: `20.0`)
* `time_scaling_factor`: Based on the estimated travel time according to the path distance and movement speed, the time_scaling_factor is a coefficient that multiplies this time by several times to extend the movement timeout. It is an empirical value. (Default: `5.0`)
