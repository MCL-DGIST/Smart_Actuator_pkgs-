# Description
This is a meta packages for Smart Actuator, including main real-time controller, GUI, MuJoco with ROS2 (Humble) interface in Nix development environment.
Also, it has a common robotics library `mcl_robotics_cpp_lib` as a submodule.

# How to collaborate
To clone this repository including submodule, 
```bash
git clone --recursive git@github.com:MCL-DGIST/mcl_quadruped_pkgs.git
```

If you edit some files or folders of submodule `mcl_robotics_cpp_lib`, then you have to add & push commit as below:
```bash
cd ./mcl_robotics_cpp_lib
git add .
git commit -m "Update library"
git push

cd ../
git add .
git commit -m "Update library"
git push
```

# Build & Run
## GUI
```bash
nix develop .#gui
start_gui
```

