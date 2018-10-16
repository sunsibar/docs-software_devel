# Troubleshooting {#docker-troubleshooting status=ready}

Symptom: `E: Failed to fetch http://packages.ros.org/ros/ubuntu/dists/xenial/main/binary-amd64/Packages  Error writing to output file - write (28: No space left on device) Error writing to file - write (28: No space left on device) [IP: 64.50.233.100 80]`

Note: Only happens in Mac since Docker is actually running VM with a fixed size

Resolution: Increase the size of your Disk image: Docker -> Preferences -> Disk and increase the slider and hit apply. 
