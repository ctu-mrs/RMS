# RMS: Redundancy-Minimizing Point Cloud Sampling

[![RMS](./fig/snapshot.jpg)](https://www.youtube.com/watch?v=Y9ZlRrX1UBY)

  * novel method for sampling large 3D LiDAR point clouds
  * replacement for voxelization
    * pipelines using **RMS** are faster (lower latency) and more accurate (less drift)
  * designed for **real-time LiDAR-based** 6-DoF odometry/SLAM pipelines
    * both point-based (ICP-like) and feature-based (LOAM-like) methods 
    * potentially improving most L, LI, LVI, LV pipelines  
  * **single parameter only -> depends on the SLAM pipeline (and not the environment!)**
    * tuned just once given your pipeline
  * deterministic (no data for learning needed) 
  * when is it (probably) not going to perform well:
    * the data have large orientation changes between two consecutive frames (tens of degrees)
    * under heavy noise (such as dust clouds)

#### Code & How to
The code will be made available upon acceptance.

#### Paper
Submitted to IEEE RA-L on December 1, 2023.
Preprint available at [arXiv](https://arxiv.org/pdf/2312.07337.pdf).

#### How to cite
```
@article{petracek2023rms,
  title   = {{RMS: Redundancy-Minimizing Point Cloud Sampling for Real-Time Pose Estimation in Degenerated Environments}}, 
  author  = {Petracek, Pavel and Alexis, Kostas and Saska, Martin}, 
  year    = {2023},
  journal = {arXiv:2312.07337},
  note    = {Submitted to IEEE RA-L on December 1, 2023}
}
```

#### Acknowledgment
To be added upon acceptance.
