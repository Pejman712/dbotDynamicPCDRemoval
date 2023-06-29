# DynamicPCDRemoval
This repository aims to provide comprehensive support to students working on projects related to point clouds for autonomous driving. It offers valuable resources, tools, and guidance to facilitate the development of innovative solutions in this field. 

# Dynamic Point Cloud Removal in PCD

Funded by DigiTally Project this reprostory aims to assist students with projects related to point cloud. 

## Table of Contents

- [Project Description](#project-description)
- [Demo](#demo)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Project Description

n recent years, indoor autonomous vehicle robotics have gained significant
popularity due to their efficiency and ability to enhance overall quality in factory
settings. These robots rely heavily on accurate maps of their surroundings for
localization. However, in highly dynamic factory environments, the mapping process
often registers dynamic points, which can negatively impact localization accuracy
in future uses. This study aims to address this challenge by aligning and merging
maps of the same location from different time periods, resulting in denser stationary
points and sparser dynamic points. To achieve this, the study proposes three filtering
algorithms: the max distance sphere method, the average distance sphere method, and
the convex hull method. These algorithms involve different approaches to estimate the
volume occupied by a point and its k (user defined) neighboring points. By repeating
this process for all points in the point cloud, a density factor is derived, which is then
used as a threshold to remove the dynamic points. To evaluate the performance of
the filtering algorithms, the study compares the odometry obtained from a merged
noisy map with its corresponding filtered map and a clean map constructed during a
period of low activity. The study also assesses factors such as computational power
requirements, reliability, and ease of implementation for each method. The findings of
the study demonstrate that the proposed filtering algorithms perform nearly as well as
a clean map in the odometry test, while the noisy map fails to accurately localize itself,
especially after high-angle turns. Among the three proposed methods, the average
distance sphere method proves to be the seemingly the most effective, primarily due to
its ability to minimize the false removal of dynamic points. 

## Demo

Example of Dynamic points removed in pcd using average sphere method is shown below.

<div>
  <img src="md1.png" alt="Dynamic Map" width="300"/>
  <img src="mf1.png" alt="Filtered" width="310"/>
</div>

## Installation

Required Libraries

```bash
$ pip install numpy
$ pip install -U scikit-learn
$ pip install scipy
$ pip install open3d 
```

## Usage  

After running DBGFA.py code, you can specify the address you want to save your filtred pcd files, the filtering method and the number of maps you use as intake and outake. 


