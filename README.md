# white_line_follower
Follows line by captured frame from camera
Code uses ros to control autonomous car. Cart_control system is control code for our autonomous car.
In this code we use "steer" to steer car, "throttle" to speed up car.
We get image and shape it to create roi (region of interest) to avoid other white object.
When we found white line it will mark it central with blue circle and follow.
