#!/bin/bash -
if ! [ -c /dev/net/tun ]; then
 mkdir -p /dev/net
 mknod -m 666 /dev/net/tun c 10 200
fi
