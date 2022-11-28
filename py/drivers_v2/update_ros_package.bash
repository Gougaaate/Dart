ROS_PKG_SRC_PATH=${HOME}/ros/dart_drivers_py27_ws/src/dartv2_drivers/src/dartv2_drivers/drivers
declare -a files=(
    "drivers_v2_encoders.py"
    "drivers_v2_imu9.py" 
    "drivers_v2_powerboard.py" 
    "drivers_v2_sonars.py" 
    "drivers_v2_trex.py" 
    "i2creal.py"
    )
echo ${files[*]}

for f in ${files[*]} 
do 
    cp -pv $f ${ROS_PKG_SRC_PATH}/
done


