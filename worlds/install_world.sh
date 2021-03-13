# Enable running this script from any directory
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

# Copy all models in /worlds/models to gazebo's model directory
mkdir -p ~/.gazebo/models
cp -vr models/circuit ~/.gazebo/models
cp -vr models/drag_race ~/.gazebo/models
cp -vr models/urban ~/.gazebo/models
