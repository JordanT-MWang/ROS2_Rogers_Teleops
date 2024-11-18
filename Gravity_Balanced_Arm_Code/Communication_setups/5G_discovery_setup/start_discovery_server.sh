#!/bin/bash
set -v
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# find 5G interface IP
DISCOVERY_SERVER_IP=$(ip -6 -o addr show | grep wwan0 | tr -s ' ' | cut -d ' ' -f 4 | cut -d '/' -f 1)
echo "Got discovery server ip: $DISCOVERY_SERVER_IP"
echo $DISCOVERY_SERVER_IP | tee /tmp/public_ip
curl -T /tmp/public_ip https://robohub.eng.uwaterloo.ca/share/teleop_discovery_server.txt
rm /tmp/public_ip

DISCOVERY_SERVER_PORT=11811
cat $DIR/fastdds.xml.template | sed "s/DISCOVERY_SERVER_IP/$DISCOVERY_SERVER_IP/" | sed "s/DISCOVERY_SERVER_PORT/$DISCOVERY_SERVER_PORT/" > $DIR/fastdds.xml

ros2 daemon stop
killall fast-discovery-server || /bin/true
echo "Now run with the correct env"
echo export FASTRTPS_DEFAULT_PROFILES_FILE=${DIR}/fastdds.xml
echo "Starting discovery server"
fast-discovery-server -x discovery_server@$DIR/fastdds.xml


