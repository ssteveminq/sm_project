# sm_project


# To record rosbag & save to csv file 
- 'SM/current_state' includes [battery state / current state])

```
rosbag record /SM/current_state 
rostopic echo -b FILE_NAME.bag -p /SM/current_state > data.csv
```
