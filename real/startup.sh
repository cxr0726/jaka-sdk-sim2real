#!/usr/bin/env bash
# JKPI: self-contained runtime with robot-network CycloneDDS configuration.
# Requires ROS 2 Humble and a running legged_waist_controller ROS node on the same DDS domain.
set +e
set +u

export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

CYCLONEDDS_XML="./config/cyclonedds.xml"
CYCLONEDDS_ENV_FILE="${CYCLONEDDS_ENV_FILE:-/opt/cyclonedds/cyclonedds_env.sh}"
CYCLONEDDS_RUNTIME_XML="${CYCLONEDDS_RUNTIME_XML:-/tmp/jkpi_cyclonedds_runtime_${ROS_DOMAIN_ID}.xml}"
# Robot network is fixed: br-ap owns 10.42.0.1/24.
# Do not bind loopback: lo makes CycloneDDS disable SPDP multicast.
CYCLONEDDS_INTERFACE_NAMES="${CYCLONEDDS_INTERFACE_NAMES:-br-ap}"
CYCLONEDDS_INCLUDE_LOOPBACK="${CYCLONEDDS_INCLUDE_LOOPBACK:-0}"
CYCLONEDDS_MAX_INTERFACES="${CYCLONEDDS_MAX_INTERFACES:-1}"
# iceoryx RouDi remains optional. Default SHM off (see jkkhan_startup.sh).
CYCLONEDDS_ENABLE_SHM="${CYCLONEDDS_ENABLE_SHM:-0}"
if [ "$CYCLONEDDS_ENABLE_SHM" = "1" ] && [ ! -e /dev/shm/iceoryx_mgmt ]; then
    _shm_wait="${CYCLONEDDS_SHM_WAIT_TIMEOUT:-15}"
    _shm_n=1
    echo "[jkpi_startup] waiting for /dev/shm/iceoryx_mgmt (${_shm_wait}s)"
    while [ "$_shm_n" -le "$_shm_wait" ] && [ ! -e /dev/shm/iceoryx_mgmt ]; do
        sleep 1
        _shm_n=$((_shm_n + 1))
    done
    if [ ! -e /dev/shm/iceoryx_mgmt ]; then
        echo "[jkpi_startup] WARNING: /dev/shm/iceoryx_mgmt missing; disabling SHM (restart iox-roudi to re-enable)" >&2
        CYCLONEDDS_ENABLE_SHM=0
    fi
    unset _shm_wait _shm_n
fi
export CYCLONEDDS_RUNTIME_XML CYCLONEDDS_INTERFACE_NAMES
export CYCLONEDDS_INCLUDE_LOOPBACK CYCLONEDDS_MAX_INTERFACES CYCLONEDDS_ENABLE_SHM

unset CYCLONEDDS_URI
unset FASTDDS_DEFAULT_PROFILES_FILE
unset FASTRTPS_DEFAULT_PROFILES_FILE

_cyclonedds_configured=0
_cyclonedds_config_source="loopback fallback"
if [ -f "$CYCLONEDDS_ENV_FILE" ]; then
    # shellcheck source=/dev/null
    if source "$CYCLONEDDS_ENV_FILE" &&
       declare -F wait_cyclonedds_interfaces >/dev/null 2>&1 &&
       declare -F setup_cyclonedds_env >/dev/null 2>&1 &&
       wait_cyclonedds_interfaces "${CYCLONEDDS_WAIT_TIMEOUT:-90}" &&
       setup_cyclonedds_env &&
       [ -n "${CYCLONEDDS_URI:-}" ]; then
        _cyclonedds_configured=1
        _cyclonedds_config_source="system setup script"
    else
        echo "[jkpi_startup] WARNING: system CycloneDDS setup failed; using loopback fallback" >&2
    fi
else
    echo "[jkpi_startup] INFO: $CYCLONEDDS_ENV_FILE not found; using loopback fallback" >&2
fi

if [ "$_cyclonedds_configured" -ne 1 ]; then
    if [ ! -f "$CYCLONEDDS_XML" ]; then
        echo "[jkpi_startup] ERROR: CycloneDDS fallback config not found: $CYCLONEDDS_XML" >&2
        exit 1
    fi
    export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
fi

_cyclonedds_config_file=""
_cyclonedds_interfaces=""
case "${CYCLONEDDS_URI:-}" in
    file://*) _cyclonedds_config_file="${CYCLONEDDS_URI#file://}" ;;
esac
if [ -f "$_cyclonedds_config_file" ]; then
    _cyclonedds_interfaces="$(
        sed -nE \
            -e 's/.*<NetworkInterface[^>]*(name|address)="([^"]+)".*/\2/p' \
            -e 's|.*<NetworkInterfaceAddress>([^<]+)</NetworkInterfaceAddress>.*|\1|p' \
            "$_cyclonedds_config_file" |
            paste -sd, -
    )"
fi

echo "[jkpi_startup] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[jkpi_startup] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[jkpi_startup] ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[jkpi_startup] CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "[jkpi_startup] CYCLONEDDS_ENV_FILE=$CYCLONEDDS_ENV_FILE"
echo "[jkpi_startup] CYCLONEDDS_INTERFACE_NAMES=$CYCLONEDDS_INTERFACE_NAMES"
echo "[jkpi_startup] CYCLONEDDS_ENABLE_SHM=$CYCLONEDDS_ENABLE_SHM"
echo "[jkpi_startup] CycloneDDS config source: $_cyclonedds_config_source"
echo "[jkpi_startup] CycloneDDS network interface(s): ${_cyclonedds_interfaces:-unknown; inspect CYCLONEDDS_URI}"
if [ "$_cyclonedds_configured" -ne 1 ]; then
    echo "[jkpi_startup] WARNING: CycloneDDS loopback fallback is active; KPI can communicate only with peers using the same host-local config" >&2
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck source=/dev/null
    if ! source /opt/ros/humble/setup.bash; then
        echo "[jkpi_startup] WARNING: Failed to source /opt/ros/humble/setup.bash" >&2
    fi
fi

# Resolve bundled legged_waist/voice_player rosidl typesupport from JKPI_PREFIX/lib first.
export AMENT_PREFIX_PATH="${JKPI_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"

# Avoid LD_LIBRARY_PATH from the shell overriding bundled libs in RUNPATH.
unset LD_LIBRARY_PATH

exec "./KpiDeployReal" "$@"
echo "[jkpi_startup] ERROR: Failed to exec KpiDeployReal" >&2
exit 1
