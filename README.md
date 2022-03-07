# Urban Sweeper Robot Planner - urbanroadsweeper
    
Use urbanroadsweeper?
If you are using urbanroadsweeper, please cite our paper **Coverage Path Planning in Large-scale Multi-floor Urban Environments with Applications to Autonomous Road Sweeping** in ?, vol. ?, no. ?, pp. ????-????, May 2022.

BibTeX:
```
@ARTICLE{8633925,
    author={D. {Engelson} and M. {Tiger} and F. {Heintz}},
    journal={?},
    title={Coverage Path Planning in Large-scale Multi-floor Urban Environments with Applications to Autonomous Road Sweeping},
    year={2022},
    volume={?},
    number={?},
    pages={????-????},
    keywords={Planning;Coverage;Cleaning;Sweeping;Mobile robots;Path planning;CPP;motion and path planning;BA*;Inward Spiral;Coverage path planning},
    doi={???},
    ISSN=={????-????},
    month={May},}
```


## System Requirements
This repository has been tested with: 
```
Ubuntu 21.04
Ubuntu 20.04
```
it might work with other setups, but no guarantees are given.

## Assumptions
1. You are running `Ubuntu 21.04` or `Ubuntu 20.04`
2. You have installed all packages in `requirements.txt` using pip3
3. You have installed `screen` using
```
sudo apt-get install screen
```
4. You run the scripts from the root directory of this repostitory

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
1. Put the files ```urban02.pcd```, ```urban05.pcd``` and ```urban17.pcd``` in the same folder.
2. Run ``` sh crop_pcd.sh PATH_TO_FOLDER ```

### 3. Run calculations
1. Configure settings for calculations in ```src/urbanroadsweeper/run_config.py```
2. For generating new start points, Run 
```
sh generate_start_points.sh ENVIRONMENT NBR_OF_START_POINTS
``` 
and replace the points in ```src/urbanroadsweeper/run_config.py```.

* ```ENVIRONMENT``` is: "garage", "bridge" or "crossing".
* ```NBR_OF_START_POINTS``` is an integer bigger than 0.

3. To run all calculations at once run ``` screen -c run_all```. 

If you want to run the calculations one by one, Run ``` sh run.sh ENVIRONMENT ALGORITHM```. 
* Where ```ENVIRONMENT``` is: "garage", "bridge" or "crossing"
* Where ```ALGORITHM``` is: :"bastar", "spiral" or "sampled"

### 4. Visualize results
1. Configure settings for visualization in ```src/urbanroadsweeper/show_config.py```
2. Run ``` sh show.sh ENVIRONMENT ALGORITHM ```
* Where ```ENVIRONMENT``` is: "garage", "bridge" or "crossing"
* Where ```ALGORITHM``` is: :"bastar", "spiral" or "sampled"

### 5. Compare results
1. Configure settings for results in "src/urbanroadsweeper/results_config.py"
2. Run ``` sh results.sh ```

## Try your own algorithm
1. Use template ```src/urbanroadsweeper/urbanroadsweeper/newCPP.py``` and add your algorithm to "get_cpp_path"
2. Add your algorithm to ALGORITHMS in ```src/urbanroadsweeper/run_config.py```. Make sure all parameters are listed in alphabetical order.
3. Add your algorithm result file to PCD_DATA in ```src/urbanroadsweeper/show_config.py``` for every environment.
4. Add your algorithm to PCD_DATA and ALGORITHM_DATA in ```src/urbanroadsweeper/results_config.py```.
5. Add screens starting your algorithm to the file ```run_all```
