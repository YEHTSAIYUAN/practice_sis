From argnctu/sis_2020:tx2

WORKDIR /home/sis/sis_competition_2020
RUN rm -rf ./catkin_ws/src/competition_modules

COPY competition_modules/ ./catkin_ws/src
COPY task.launch ./catkin_ws/src/core/sis_task/launch/
COPY catkin_make.sh .
COPY environment.sh .
COPY run_task.sh .

RUN /bin/bash -c "cd ~/ && source /opt/ros/melodic/setup.bash"
