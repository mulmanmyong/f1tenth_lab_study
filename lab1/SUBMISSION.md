# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: `source /opt/ros/foxy/setup.bash` and `source install/local_setup.bash`. Functionally what is the difference between the two?

Answer: (FILL ME IN)\
전자는 ROS2 foxy 시스템 설치 환경 설정에 관한 것이고, 후자는 현재 작업 중인 ROS2 워크스페이스 환경 설정에 관한 것이다. 이는 기본 설치된 환경 위에 덮어 쓴다.

### Q2: What does the `queue_size` argument control when creating a subscriber or a publisher? How does different `queue_size` affect how messages are handled?

Answer: (FILL ME IN)\
queue_size는 publisher 또는 subscriber의 메시지 버퍼 크기를 의미한다.\
작으면 메시지 손실 가능성 높고, 메모리 사용이 적고, 크면 메시지 처리 버스트 처리가 가능하고 메모리 사용이 많다.

### Q3: Do you have to call `colcon build` again after you've changed a launch file in your package? (Hint: consider two cases: calling `ros2 launch` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: (FILL ME IN)\
첫번째 경우는 launch 파일 위치에서 ros2 launch를 실행할 경우인데, 이 경우에서는 수정을 해도 다시 빌드할 필요가 없다. 수정을 해도 파일 위치에서 실행을 하는 것이기 때문에 변경 내용이 적용된다.\
두번째 경우는 설치된 패키지에서 인데 이때는 다시 빌드를 해야한다. 설치된 패키지에서 launch를 실행을 하면 변경되어도 다시 빌드를 해줘야지 새로운 파일로 업데이트가 되기 때문에 다시 빌드를 해서 변경된 내용으로 설치를 하고 실행을 해줘야 한다.
