echo "Welcome to Point cloud cropper"
echo "Argument 1: Path of folder with the pointclouds"
echo "Expecting files urban02.pcd, urban05.pcd and urban17.pcd."
echo "Starting with Crossing. File: urban17.pcd."
python3 src/exjobb/exjobb/crop_pcd.py $1 urban17.pcd 326608.680 4152387.95 48.8488709 50 crossing.pcd