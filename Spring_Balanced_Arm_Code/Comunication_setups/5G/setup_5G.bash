source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
export FASTRTPS_DEFAULT_PROFILES_FILE=${DIR}/fastdds.xml

