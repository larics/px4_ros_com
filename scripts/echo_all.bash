#!/bin/bash

# 1. List all ROS 2 topics
echo "Listing all ROS 2 topics..."
ros2 topic list > topics.txt
echo "Saved topics to topics.txt"

# 2. Echo each topic for 2 seconds
echo "Echoing each topic for 2 seconds..."
while read -r topic; do
    echo "Echoing: $topic"
    timeout 2s ros2 topic echo "$topic"
    echo "----"
done < topics.txt

echo "Done."
