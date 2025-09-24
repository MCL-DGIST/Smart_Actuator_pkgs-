## This defines a function which returns a packge, intended to be evaluated by callPackage
{ lib
, buildRosPackage
, rosidl-default-runtime
, rosidl-default-generators
, builtin-interfaces
# , std-srvs
, std-msgs
, sensor-msgs
, geometry-msgs
}:

buildRosPackage {
  pname = "mcl-quadruped-msgs";
  version = "0.0.0";
  
  src = ./.;
  
  buildType = "ament_cmake";

  ## These are the build-time dependencies
  nativeBuildInputs = [
    rosidl-default-generators
  ];

  ## These dependencies are propagated, any package (or shell) which
  ## includes these will also include these deps
  propagatedBuildInputs = [
    rosidl-default-generators
    rosidl-default-runtime
    builtin-interfaces
    # std-srvs
    std-msgs
    sensor-msgs
    geometry-msgs
  ];

  meta = {
    description = "Custom messages for ELMO Servo Control";
  };
}
