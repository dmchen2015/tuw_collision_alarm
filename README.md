# tuw_collision_alarm

### How to use:
There are 2 launch files to start this module:

* ```roslaunch tuw_collision_alarm tuw_collision_alarm``` 
starts the nodelet via the nodelet manager directly.


* ```roslaunch tuw_collision_alarm tuw_collision_alarm_node``` 
starts the nodelet via another node for easy debugging.

There are 2 parameters that can be set via the launch files:

* ```distance_threshold``` which defines the minimum distance to the _Path_. It defaults to 0.2 meters.

* ```obstacle_vote_threshold ``` which defines the number of minimum laser beam end points within the ```distance_threshold``` for something to be considered an obstacle. It defaults to 5.

### What does it do:

This module checks alongside the _Path_ for possible obstacles and in the case of finding one. it publishes an index of the _Path_ that comes before the obstacle. If there are no obstacles, the very last index is published. The name of the topic is called _collision_alarm_index_. 

### How does it do it:
It iterates over the _Path_, creates line segments and check distance of laser beam endpoints to see if there is something that can be considered an obstacle defined by the parameters above. 

### TODO

At the moment this module uses its own implementation to check where it is on the _Path_. In the future, this feature will be replaced by the information provided by another module.

