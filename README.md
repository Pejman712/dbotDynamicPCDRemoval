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

SLAM (Simultaneous Localization and Mapping) has gained popularity in recent years as it addresses the challenges of both localization and mapping simultaneously. One key issue in mapping is the registration of false points in dynamic environments, which can have a negative impact on localization accuracy. Existing studies often require highly dynamic environments, are computationally expensive, and have a high probability of false detection. Furthermore, many of these studies have primarily focused on outdoor environments. To overcome these challenges, this study aims to propose efficient and reliable filtering algorithms in indoor environments, namely the max distance sphere method, average distance sphere method, and Convexhull. To evaluate the performance of the algorithms, the study compares the odometry obtained from a merged noisy map (constructed by aligning maps from four different times) captured during an annual event at Aalto University called PDP, with its filtered map and a clean map constructed during a deserted period. Furthermore, the study assesses the computational power required, reliability, and ease of implementation for each method. The results of this study demonstrate the effectiveness of the proposed filtered algorithms in achieving engineering-level localization accuracy. These algorithms have potential applications in various settings such as factories, construction sites, and digital twin applications.

## Demo

Example of Dynamic points removed in pcd using average sphere method is shown below.

<div>
  <img src="md1.png" alt="Dynamic Map" width="300"/>
  <img src="mf1.png" alt="Filtered" width="320"/>
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


