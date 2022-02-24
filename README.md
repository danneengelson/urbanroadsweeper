# urbanroadsweeper

## Installation and execution

### 1. Download the Point Clouds

Request download links from https://sites.google.com/view/complex-urban-dataset/download-lidar?authuser=0

* Garage environment: urban05
* Bridge environment: urban02
* Crossing environment: urban17

Locate .las-file and use https://github.com/murtiad/las2pcd to convert it to .pcd

Example:
```
las2pcd urban05_las/urban05/sick_pointcloud.las urban05.pcd
```

#### Troubleshooting

* E: Package 'liblas-dev' has no installation candidate
* E: Unable to locate package liblas-c-dev

Download and compile the latest liblas version by following instructions "Using Unix Makefiles on Linux" at https://liblas.org/compilation.html

To make "lasinfo" work you might need to run following:
```
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export LD_LIBRARY_PATH
cd las2pcd
cmake .
```

To make "make" work you might need to 
1. Change to local in CMakeLists.txt in las2pcd:
```
/usr/**local**/lib/liblas.so
/usr/**local**/lib/liblas_c.so
```
2. Run following:
```
mv las2pcd /usr/bin/
```

### 2. Crop Point Clouds
1. Put the files "urban02.pcd", "urban05.pcd" and "urban17.pcd" in the same folder.
2. Run ``` sh crop_pcd.sh PATH_TO_FOLDER ```

### 3. Run calculations
1. Configure settings for calculations in "src/exjobb/run_config.py"
2. Run ``` sh run.sh ENVIRONMENT ALGORITHM```. 
**Where ENVIRONMENT is: "garage", "bridge" or "crossing"
**Where ALGORITHM is: :"bastar", "spiral" or "sampled"

### 4. Visualize results
1. Configure settings for visualization in "src/exjobb/show_config.py"
2. Run ``` sh show.sh ENVIRONMENT ALGORITHM ```
**Where ENVIRONMENT is: "garage", "bridge" or "crossing"
**Where ALGORITHM is: :"bastar", "spiral" or "sampled"

### 5. Compare results
1. Configure settings for results in "src/exjobb/results_config.py"
2. Run ``` sh results.sh ```