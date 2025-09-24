{
  description = "Meta-packages for MCL Legged Robot Control";

  inputs = {
    nixpkgs_2411.url = "github:nixos/nixpkgs/nixos-24.11";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # ! IMPORTANT 
    nixgl.url = "github:nix-community/nixGL";
  };

  outputs = { self, nixpkgs, nixpkgs_2411, nix-ros-overlay, nixgl }:
    let
      system = "x86_64-linux";
      
      overlays = [
        nix-ros-overlay.overlays.default
        nixgl.overlay
      ];
      
      pkgs = import nixpkgs {
        inherit system overlays;
      };

      rospkgs = pkgs.rosPackages.humble;
      rosPackages = with rospkgs; [
        rclcpp
        ros-core
        ament-cmake-core
        
        std-msgs
        sensor-msgs
        geometry-msgs

        joy
        rosbag2
      ];

      mylib = pkgs.callPackage ./mylib {};
      mcl-quadruped-msgs = rospkgs.callPackage ./msgs {};
      mcl-quadruped-gui = rospkgs.callPackage ./gui {
        inherit mylib mcl-quadruped-msgs;
      };

      projectPackages = [
        mylib
        mcl-quadruped-msgs
      ];
      
      guiPackages = [
        mcl-quadruped-gui
        pkgs.nixgl.nixGLIntel
      ];
      


    in
      {
        devShells.${system} = {
          default = pkgs.mkShell {
            packages = [
              mylib
              pkgs.colcon
              pkgs.libbpf

              # ... other non-ROS packages

              (rospkgs.buildEnv {
                paths = rosPackages
                        ++ projectPackages
                        ++[];
              })
            ];
          };
# ++ projectPackages
          gui = pkgs.mkShell {
            shellHook = ''
              alias start_gui="nixGLIntel ros2 run mcl_quadruped_gui mcl_quadruped_gui --force-discover"
              alias qt_designer='QT_QPA_PLATFORM=xcb nixGLIntel designer'
              export PS1="(gui) $PS1"
            '';

            packages = [

              pkgs.qt5.qttools

              (rospkgs.buildEnv {
                paths = rosPackages 
                        ++ guiPackages;
              })
            ];
          };

          sim = pkgs.mkShell {
            shellHook = ''
              alias start_sim="nixGLIntel ./main"
              export PS1="(sim) $PS1"
            '';

            packages = [
              pkgs.mujoco 
              pkgs.glfw
              pkgs.nixgl.nixGLIntel
              

              pkgs.pinocchio
              pkgs.crocoddyl

              (rospkgs.buildEnv {
                paths = rosPackages;
              })
            ];
          };


        };
      };

  nixConfig = {
    bash-prompt-prefix = "\\[\\e[38;5;12m\\](nix)\\[\\e[38;5;220m\\] ";

    extra-substituters = [
      "https://ros.cachix.org"
      "https://nix-community.cachix.org"
    ];
    
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
    ];
  };
}
