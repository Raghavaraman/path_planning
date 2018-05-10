# CarND-Path-Planning-Project
### Self-Driving Car Engineer Nanodegree Program
 
The goal of this project was to control the speed and lane of a car given the sensor fusion data of cars around me via the simulator. The steps that I followed for the development were as follows:

1. I made the car go at a speed less than the speed limit of 50mph.
2. I Made it go in the current line as long as there was no car in front of it 
3.  when it identifies a car ahead it slows down and at the same time checks if any other lane is relatively free.
4. If a near by lane is free, it would first attempt changing the lane.
5. In case all the near by lane are crowded too, it will slow down in the present lane.

# Rubic Criteria:

### The code compiles correctly.
Yes. The code complies when we use cmake and make 

### The car is able to drive at least 4.32 miles without incident..
Yes, I test ran it for upto 7 miles.

### The car drives according to the speed limit.
The car reaches a speed of 49.5 mph at most.

### Max Acceleration and Jerk are not Exceeded.
The car never jerks or accelerates/decelerates too fast. (Thanks to Spline library)

### Car does not have collisions.
In my test run, it didn't have an collision or any other traffic/comfort violations 

### The car stays in its lane, except for the time between changing lanes.

The car is sticky to the lanes.

### The car is able to change lanes
The car changes lanes whenever the other lanes are relatively free.

### There is a reflection on how to generate paths.

# Model Documentation

Generation of the three key future points of the car, this is used by the spline library as reference

```
vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

To fill in the gaps using spline library, we do the following:

```
// Calculate how to break up spline points so that we travel at our desired
// reference velocity and avoid jerk violation.
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
```

### Finally to fill in the paths points using spline, we do the following:

```
// Fill up the rest of our path planner after filling it with previous points, here we
// will always output 50 points.
for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

    double N = (target_dist/(0.02*ref_vel/2.24)); // 0.5 mph
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;
}
```

Finally the logic to change lanes if handled as follows:

```

if(too_close){
	ref_vel -= 0.224;
	if (( lane == 1) && !left_too_close){
	lane = 0;
}
else if (( lane == 1) && !right_too_close) {
	lane = 2;
}
else if (( lane == 0) && !right_too_close){
	lane = 1;
}
else if (( lane == 2) && !left_too_close){
	lane = 1;
}
}
else if (ref_vel < 49.5){
           ref_vel += 0.224;
}

```



