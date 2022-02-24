echo "Welcome to Point cloud cropper"
echo "Argument 1: Path of folder with the pointclouds"
echo "Expecting files urban02.pcd, urban05.pcd and urban17.pcd."
echo "Garage environment. File: urban05.pcd."
python3 src/exjobb/script/crop_pcd.py $1 urban05.pcd 351490 4022890.5 58 30 garage.pcd
echo "Crossing environment. File: urban02.pcd."
python3 src/exjobb/script/crop_pcd.py $1 urban02.pcd 326679 4152494 58 50 crossing.pcd
echo "Bridge environment. File: urban17.pcd."
python3 src/exjobb/script/crop_pcd.py $1 urban17.pcd 318243.375 4156433.7 19 250 bridge.pcd bridge
