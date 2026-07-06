# VDA5050 Library and Support Tools

This repository provides libraries and tools to
- Enable VDA5050 compatibility for AGV/AMRs
- Build custom VDA5050 Master.

# Build and install 
```bash 
colcon build 
echo "source ${PWD}/install/setup.bash" >> ~/.bashrc
```

# Run C++ Adapter
```bash
ros2 run vda5050_core adapter_example tcp://localhost:1883 my_agv
```
# Run AutoXing Demo 
we will be sending a MQTT order message 4 base to follow. 
MQTT message -> VDA5050-Adapter-CPP -> Python-Bindings -> Autoxing-Python-Client -> Autoxing-RESTAPI-Interface 
```bash 
cd vda5050_core

# build the vda5050, this will install the vda5050_core_py to your default python3
colcon build --symlink-install
```

```bash 
## Terminal 1
# start the python exmaple client, can use as a guide for building your own client using the vda5050_core_py 
cd vda5050_core
/usr/bin/python3 vda5050_core_py/autoxing-l300/example_autoxing_client.py

## Terminal 2
# Sending the MQTT message to the broker
cd vda5050_core
/usr/bin/python3 vda5050_core_py/autoxing-l300/publish_autoxing_l300_route.py
```


# Troubleshoot 
if fail try adding `Add: [-std=c++17]` to the .clangd compileFlag

  