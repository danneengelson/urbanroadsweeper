echo "Welcome to Point cloud cropper"
echo "Argument 1: Path of folder with the pointclouds"
echo "Expecting files urban05.pcd and urban17.pcd."
echo "Garage environment. File: urban05.pcd."
python3 src/exjobb/exjobb/crop_pcd.py $1 urban05.pcd 351490 4022890.5 58 30 garage.pcd
echo "Crossing environment. File: urban17.pcd."
python3 src/exjobb/exjobb/crop_pcd.py $1 urban17.pcd 326608.680 4152387.95 48.8488709 50 crossing.pcd'
