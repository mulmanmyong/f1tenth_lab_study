# F1TENTH LAB study repo

## 해당 REPO를 만든 이유

- 팀원이 이미 진행한 코드가 존재, 하지만 **f1tenth lab**을 봤을 때, 해당 코드를 직접 작성하는 방식임을 몰랐음
- 그저 그냥 해당 코드에 대한 설명만 있고, 실행을 해보는 시간으로 착각함
- 이를 lab7을 진행할 때 **assignment**라는 것이 존재하고, 이로 인해 점수가 존재함을 확인함
- 그래서 이미 아는 내용이어도 해당 레포에 clone을 하여 진행하는 것으로 결정

## LAB1

**LAB1은 ROS2에 대한 기본 지식에 대해 알아보는 시간이다**

### 목표

- 목표로는 ROS2에 익숙해지고, 어떤 식으로 **publisher**와 **subscriber** node를 생성하는 지 이해하는 시간이다
- ROS2의 패키지 구조, 파일, 의존성들에 대해서 이해하는 시간이다
- launch 파일을 생성해보는 시간이다

### ROS2 기본

- ROS2 사용을 위한 명령

```bash
source /opt/ros/foxy/setup.bash
```

해당 명령을 **terminal**에 입력을 하면 ROS2 명령어를 사용할 수 있게
됨

- 기본 명령어

```bash
ros2 topic list
```

을 통해 **topic list**를 볼 수 있는데, 기본적으로 아래 두 개의 토픽을 볼 수 있음

```bash
/parameter_events
/rosout
```

### Package 생성

- ROS2 패키지는 'Python'과 'C++' 둘 다 지원함
- 이 때 패키지를 생성할 때 패키지 이름뿐만 아니라 build type을 지정할 수도 있고, 의존성도 지정할 수 있음
- 주로 'C++'로 이용할 것이기 때문에 **build type**은 'CMake'로 지정
- 패키지 이름은 'lab1_pkg'로 생성
- 의존성과 관련된 부분은 'package.xml'에서 선언이 가능하며, build type 또한 'package.xml'에서 변경 가능함
- 해당 패키지는 의존성으로 'ackermann_msgs'를 필요로 함
- 여기까지 수행과정은

```bash
ros2 pkg create lab1_pkg
or
ros2 pkg create --build-type ament_cmake lab1_pkg
or
ros2 pkg create --build-type amnet_python lab1_pkg
or
ros2 pkg create --build-type ament_cmake lab1_pkg --dependencies ackermann_msgs
```

첫번째는 기본적인 패키지 생성 방법이고, **build type**은 기본적으로 **cmake**이며 의존성도 선언되어 있지 않음
두번째는 **build type**을 **cmake**로 지정해주는 것이고, 이도 동일하게 의존성은 선언되어 있지 않음
세번째는 **build type**을 **python**으로 지정해주는 것이고, 이도 동일하게 의존성은 선언되어 있지 않음
네번째는 **build type**을 **cmake**으로 지정해주고, 의존성에 **ackermann_msgs**를 선언해줌

- 결론적으로 네번째 방식을 사용을 하면 요구사항을 'package.xml'에서 추가를 안해줘도 됨
- 'package.xml'에서 충분히 수정 가능하니 첫번째 방식을 사용하거나 오타가 나도 무방함
- 선언한 의존성이 설치될 수 있도록 'rosdep' 명령어를 사용

```bash
rosdep install --from-paths src --ignore-src -r -y
```

- 패키지 내부에는 중복된 'src' 폴더나 불필요한 'install'이나 'build'가 없어야 함

### Publisher와 Subscriber 노드 생성

- 언어는 'C++' 또는 'Python' 중 원하는 것으로 작성하기
- 여기서는 'C++' 로 먼저 작성

- **첫번째 노드**
- 'talker.cpp' 또는 'talker.py'로 작성
- 'talker'는 'v'와 'd' 두 개의 ROS 파라미터 수신
- 'talker'는 'speed' 필드와 'v' 파라미터가 같고, 'steering_angle' 필드와 'd' 파라미터가 같은 'AckermannDriveStamped' 메시지를 'drive'라는 이름의 토픽으로 **publish**

- **두번째 노드**
- 'relay.cpp' 또는 'relay.py'로 작성
- 'relay'는 'drive' 토픽을 **subscribe**
- 받은 메시지인 'speed'와 'steering_angle'을 3배 해서 'AckermannDriveStamped' 메시지를 'drive_relay'라는 이름의 토픽으로 **publish**

### Launch 파일 생성

- 위에서 작성한 **publisher**와 **subscriber**를 확인하기 위해서 launch 파일을 작성해야 함
- launch 파일 툴을 보고 그대로 작성하였으나 실패 -> 공부가 부족했다는 증거
- launch폴더를 인식하기 위해서 CMakeLists.txt에서 install 경로에 launch 폴더를 추가하여 인식할 수 있도록 해야 함 (단, ament_package() 위에)

```txt
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

- 그 결과 launch 파일 정상 인식 후, 실행까지 가능. 하지만 여기서 다양한 에러 발생. 에러 분석 후 수정
- 이번에는 실행을 하고자 하는 node들을 인식하지 못하는 것에 대한 문제가 발생함.
- 이도 launch를 인식할 수 있도록 작성한 **cpp** 코드를 인식 할 수 있도록 해야함

```txt
# Include directories
include_directories(include)

# Add executables
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp ackermann_msgs)

add_executable(relay src/relay.cpp)
ament_target_dependencies(relay rclcpp ackermann_msgs)

# Install executables
install(TARGETS
  talker
  relay
  DESTINATION lib/${PROJECT_NAME})
```

- 그 결과 정상 작동함을 알 수 있음

### LAB1 결론

- 주의해야할 점이 존재
- 기존에 무작정 있는 코드를 작성하니 뭐가 필요한 지 잘 몰랐음
- install에 정상적으로 설치가 되어야 **ROS**에서 인식할 수 있으므로, 이와 관련 된 부분을 특히 주의해야함
- install구문을 CMakeLists.txt에 추가하는 법을 까먹을 시 **'Launch 파일 생성'** 부분을 다시 볼 것
- 2024/7/20 기준으로 아직 **python**에 관한 것은 작성안함

### ROS 2 커맨드

- 알아두면 쓸 일이 있는 ROS2 커맨드 topic과 node가 정상적으로 작동하고 송수신 하는 지 확인할 때 주로 사용

```bash
ros2 topic list
ros2 topic info drive
ros2 topic echo drive
ros2 node list
ros2 node info talker
ros2 node info relay
```

### 패키지 구성

- 패키지 구성

```txt
- package.xml (패키지 설정 파일)
- CMakeLists.txt (빌드 설정 파일)
- setup.py (파이썬 패키지 설정 파일)
- setup.cfg (파이썬 패키지 환경 설정 파일)
- plugin.xml (RQt 플러그인 설정 파일)
- CHANGELOG.rst( 패키지 변경로그 파일)
- LICENSE (라이선스 파일)
- README.md (패키지 설명 파일)
```
