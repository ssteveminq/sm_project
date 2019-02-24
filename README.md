# sm_project

### To launch human checker
```
ssh_backpack
roslaunch gaze_service checkperson_backpack.launch
```

### launch villa_manipulation action servers & Obstacle checker
```
roslaunch villa_manipulation hri.launch
```

### run slugmanager action servers
```
rosrun sm_project slugmanager.py
```
### run smach state machie with HSR
```
rosrun sm_project hsr_state_machine.py
```


### To record rosbag & save to csv file 
- 'SM/current_state' includes [battery state / current state])

```
rosbag record /SM/current_state 
rostopic echo -b FILE_NAME.bag -p /SM/current_state > data.csv
```
