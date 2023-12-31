#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec rosrun wheels_package wheels_node.py &  # line_detector_node.py
#dt-exec rosrun line_detector_package line_detector_node.py # line_detector_node.py
dt-exec rosrun camera_package edge_detection_node.py # line_detector_node.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
