{ lib
, stdenv
, cmake
, fetchurl
, yaml-cpp
, pinocchio
, eigen
, urdfdom-headers
, urdfdom
}:

let
  # build SOEM from tar.gz
  soem = stdenv.mkDerivation rec {
    pname = "soem";
    version = "1.4.0";

    src = fetchurl {
      url = "https://github.com/OpenEtherCATsociety/SOEM/archive/v${version}.tar.gz";
      sha256 = "0vnq264yirhv12l01b4ycg55d89g9nrhmhfjfr5fibbd4461x3p5";
    };

    nativeBuildInputs = [ cmake ];

    # check directory structure after decompression
    preConfigure = ''
      echo "Source directory contents:"
      ls -la
      pwd
    '';

    cmakeFlags = [
      "-DBUILD_TESTS=OFF"
      "-DCMAKE_BUILD_TYPE=Release"
      "-DCMAKE_C_FLAGS=-Wno-error=stringop-truncation -Wno-stringop-truncation"
      "-DCMAKE_CXX_FLAGS=-Wno-error=stringop-truncation -Wno-stringop-truncation"
    ];

    # check after installation
    postInstall = ''
      echo "SOEM installation complete:"
      find $out -type f | head -10
    '';
  };

in

stdenv.mkDerivation {
  pname = "mylib";
  version = "0.0.0";
  src = ./.;

  nativeBuildInputs = [
    cmake
  ];

  buildInputs = [
    soem
    yaml-cpp
    pinocchio
    eigen
    urdfdom-headers
    urdfdom
  ];

  propagatedBuildInputs = [
    soem
    yaml-cpp
    pinocchio
    eigen
    urdfdom-headers
    urdfdom
  ];

  preConfigure = ''
    export SOEM_PATH=${soem}
    export PINOCCHIO_PATH=${pinocchio}
    echo "Using SOEM from: ${soem}"
  '';

  cmakeFlags = [
    "-DSOEM_DIR=${soem}"
    "-DPINOCCHIO_DIR=${pinocchio}"
  ];

  meta = {
    description = "Common libraries for other packages";
  };
}
