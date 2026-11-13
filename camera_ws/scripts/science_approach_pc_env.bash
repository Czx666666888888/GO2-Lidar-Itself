# Source ON the PC before the remote-camera/read-only SLAM/approach launches.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ "${GO2_RUNTIME_PROFILE:-}" != local ] && [ -z "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enp12s0</NetworkInterfaceAddress></General><Discovery><Peers><Peer Address="192.168.123.18"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
fi
