# 이전 답변의 간단한 CMakeLists.txt 예시
cmake_minimum_required(VERSION 3.16)
project(MujocoExample CXX)

# 1. "Nix가 설치해준 MuJoCo 라이브러리 찾아줘!"
find_package(mujoco REQUIRED)

# 2. "내가 만든 main.cpp로 'main' 실행 파일 만들어줘."
add_executable(main main.cpp)

# 3. "만들어진 'main'이 MuJoCo 라이브러리를 사용하도록 연결해줘."
target_link_libraries(main PRIVATE mujoco::mujoco)