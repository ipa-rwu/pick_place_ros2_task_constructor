# Pick place application ros2

## start commands
### Define object pose and place pose in "config/pick_place_demo.yaml"
```
// you can change paramters in "config/pick_place_demo.yaml"
ros2 launch pick_place_app panda_mtc.launch.py

ros2 launch pick_place_app pick_place_demo.launch.py
```

### Send object pose and place pose by service call
```
ros2 launch pick_place_app panda_mtc.launch.py

ros2 launch pick_place_app pick_place_server.launch.py

// call server
ros2 launch pick_place_app call_pick_place_server.launch.py
```
