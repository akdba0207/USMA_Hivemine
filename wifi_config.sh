#!/bin/bash

usage()
{
cat <<EOF
Usage: $0 [options] wireless-device id-number
Options:
    -P POWER        Set txpower (default=10, set to 20 for flight ops)
EOF
}

# option defaults
INET_CONFIG=0
ROUTER_USE=0
ROUTER_DEV=""
TXPOWER=1000

# Network defaults
TEAM_SUBNET=11
TEAM_CHAN=1
TEAM_AP="00:99:88:77:66:55"
TEAM_SSID="army"

#parse options
while getopts ":2IR:P:T:h" opt; do
    case $opt in
        P)
            if [ -z $OPTARG ]; then
                usage
                exit 1
            fi
            TXPOWER=$OPTARG
            ;;
        h)
            usage
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

if [ -z $1 ] || [ -z $2 ]; then
  usage
  exit 1
fi

echo "setting up for $TEAM_SSID ..."

if [ $ROUTER_USE != 0 ]; then
  echo "Setting up router ..."
  sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
  sudo iptables -t nat -F  # Flush old rules first
  sudo iptables -t nat -A POSTROUTING -o $ROUTER_DEV -j MASQUERADE
fi

# Make sure SITL bridge device isn't running
for dev in `/sbin/ip link | grep sitl_bridge | grep ',UP,' | awk '{print $2}' | sed 's/://'`; do
  echo "Disabling interface $dev ..."
  sudo ip link set $dev down
done

sudo ip link set $1 down
sudo iw dev $1 set type ibss
sudo ip addr add 192.168.$TEAM_SUBNET.$2/24 broadcast + dev $1
sudo ip link set $1 up
sudo iw dev $1 ibss join $TEAM_SSID 2412 $TEAM_AP
sudo iw dev $1 set txpower fixed $TXPOWER

if [ $INET_CONFIG != 0 ]; then
  sudo route add default gw 192.168.${TEAM_SUBNET}.1
  echo -e "nameserver 172.20.20.11\nnameserver 8.8.8.8" | sudo tee /etc/resolv.conf
fi
