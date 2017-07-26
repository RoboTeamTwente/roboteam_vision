#!/bin/bash

if [ $# -lt 2 ]; then
  echo "Usage: change_vision_ports.sh (vision|referee) <port>"
  echo "Or:    change_vision_ports.sh both <vision_port> <ref_port>"
  exit
fi

case $1 in
  'vision')
    paramcmd="rosparam set vision_source_port $2"
    ;;
  'referee')
    paramcmd="rosparam set referee_source_port $2"
    ;;
  'both')
    if [ $# -lt 3 ]; then
      echo "Need 2 ports"
    else
      paramcmd="rosparam set vision_source_port $2 && rosparam set referee_source_port $3"
    fi
    ;;
  *)
    echo "Unkown command"
esac

eval $paramcmd
