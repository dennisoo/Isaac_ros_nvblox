#!/bin/bash
# stop_container.sh - Stoppt den Isaac ROS Container für den aktuellen Benutzer

CONTAINER_NAME="isaac_workspace"

echo "🛑 Stoppe Container: $CONTAINER_NAME"

# Prüfe ob Container läuft
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container wird gestoppt..."
    docker stop $CONTAINER_NAME
fi

# Prüfe ob Container existiert (auch gestoppt)
if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container wird entfernt..."
    docker rm $CONTAINER_NAME
fi

echo "✅ Fertig. Container '$CONTAINER_NAME' wurde gestoppt und entfernt."