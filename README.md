This is a ROS1 workspace to the object fusion algorithm based on Aeberhard (2017) http://dx.doi.org/10.17877/DE290R-18029 https://eldorado.tu-dortmund.de/handle/2003/36011


This work is a simplified implementation of the proposed fusion architecture below.
![Aeberhard proposed Fusion Architecture(Aeberhard(2017))](imgs/aeberhard_fusion_level.png)



Install dependencies:

```bash
pip install numpy==1.21.2 rospkg==1.3.0 scipy==1.7.1
```

Compile the workspace

```bash
catkin build
```

Execute the code with:

```bash
rosrun object_fusion fusion.py
```

Visualize the output data (fused object markers) in RVIZ with:

```bash
rosrun object_fusion visualization_fusion.py
``` 

The fusion input in this example uses two subscribers, but it is not limited to those listes, any sensor with the same ROS1 message is accepted. The subscriber can be found in the [fusion.py file](src/object_fusion/src/fusion.py) as seen below.

```python
    camera_subscriber = rospy.Subscriber("/fusion/input/sensors/camera", Object_List, callback_handler.callback)
    radar_subscriber  = rospy.Subscriber("/fusion/input/sensors/radar" , Object_List, callback_handler.callback)    

```


(TO DO:)In case you use this code, please cite it with: 
```
@INPROCEEDINGS{10066615,
  author={Poledna, Yuri and Reway, Fabio and Drechsler, Maikol Funk and Huber, Werner and Icking, Christian and Ribeiro, Eduardo Parente},
  booktitle={2022 10th International Conference in Software Engineering Research and Innovation (CONISOFT)}, 
  title={An Open-Source High-Level Fusion Algorithm in ROS for Automated Driving Applications}, 
  year={2022},
  volume={},
  number={},
  pages={174-181},
  doi={10.1109/CONISOFT55708.2022.00031}}
```
