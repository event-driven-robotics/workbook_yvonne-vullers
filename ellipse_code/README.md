# ELLIPSE_CODE
***NOTE: in the code (config.ini, main.cpp, step-by-step.cpp, affine.h), theta and phi are switched compared to the paper. ***

To run this code, first load the dataset into **yarpdataplayer**. Make sure that the user and eye in [step-by-step.cpp](/ellipse_code/step-by-step.cpp) and [main.cpp](/ellipse_code/step-by-step.cpp) correspond to the user in the data. 
The tracking can then be performed in two ways. Either step-by-step, or real-time.

## step-by-step
The step-by-step tracking is performed by running [step-by-step.cpp](/ellipse_code/step-by-step.cpp). For every packet of data, the code is run. The next packet of data is only fetched when the code has finished completely. The code starts running as soon as the the data is played.

From the [config file](/ellipse_code/config.ini) the start orientation (theta, phi, radius, eyeball center), eyelid parameters (to define the polynomials), and the begin and end time for tracking are fetched for the defined user.

The tracking output contains:
- the timestamp 
- the coordinates defining the estimated bounding box around the iris
- a 1, used to compare the bounding box to the ground truth in mustard
- the coordinates of the center of the iris
- the latency of the iteration in ms
This data is written in the file [center.csv](ellipse_code/center.csv).

Additionally, some parameters can be changed to change the tracking and/or visualization:
- *fast*: this parameter determines wheter extra visualitzations (e.g. for debugging) are shown. Setting this value to true will give the fastest performance.
- *use_eros*: setting this parameter to true means the EROS is used for tracking. Otherwise a fixed temporal window is used to represent the data.
- *colored*: setting this parameter to true will show the tracked ellipse in red.
- *video*: a video of the tracking will be made if this parameter is true.

## real-time
The real-time tracking code in [main.cpp](/ellipse_code/step-by-step.cpp) performs the tracking while continuosly fetching the data, i.e. it does not wait until the code is finished. 

From the [config file](/ellipse_code/config.ini) the start orientation (theta, phi, radius, eyeball center) and eyelid parameters (to define the polynomials) are fetched for the defined user.

The tracking output contains:
- the timestamp 
- the coordinates defining the estimated bounding box around the iris
- a 1, used to compare the bounding box to the ground truth in mustard
- the coordinates of the center of the iris
- the latency of the iteration in ms
This data is written in the file [center.csv](ellipse_code/center.csv).

We can now only change the *fast* parameter for a faster implementation of the tracking. 

Additionally, this code does not automatically start running the code. First, the data should be played, then the ***G*** key should be pressed at the right time to start tracking (implementing an automatic start like in the step-by-step code could be considered).

## affine.h
Both methods of tracking use [affine.h](/ellipse_code/affine.h) for tracking. This file contains the functions to create the templates, compare the templates to the EROS, and keep track of the current state.

## config.ini
In this file you can find the configurations for each user. For users 5-0, 5-1, 13-0, 13-1, 18-0, 18-1, 19-0, 19-1 the configurations are correct. For other users the configuration should be improved (2-0, 10-0, 15-0, 15-1) or defined and included.