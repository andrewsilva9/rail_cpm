# rail_cpm

Notably lacking the models directory (and requiring an absolute path to that missing directory). Contact me to get the model, since it's too big to push up with everything else.
This also expects that caffe is in your $PYTHONPATH, so if it is not then ensure you do the sys.path workaround.

### What is published?
This node will publish /rail_cpm/keypoints, which is a list of lists. Each big list is a person, within that there are named lists like 'nose', 'neck', 'left_ear', etc. and each of these correspond to (Y, X) coordinates of that particular feature on that person. For a visualization, run with the debug flag and run:
```
rosrun image_view image_view image:=/rail_cpm/debug/keypoint_image
```

### Running:
To actually run the node, launch with:
```
roslaunch rail_cpm detector.launch
```
optional flags include 
  * 'debug' (true or false)
  * 'image_sub_topic_name' which is /kinect/qhd/image_color_rect by default.


### Editing things:
If you want to change the keypoints that are returned, you need to mess around with lines 218-237. I have hard-coded the limb-sequences that I am interested in, which means that knees, hips, and ankles aren't coming through. Currently, the code is going
```
for i in range(6):
  ...
for i in range(12, 19):
  ...
```
If you want to get all keypoints, all you need to do is cut out one for loop and extend the range of the other, for example:
```
for i in range(19):
  ...
```
And actually, 18 might work too...
For other questions, just reach out!
