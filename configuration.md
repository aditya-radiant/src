# Configuration
1. `mkdir Bascorro_ws`
2. `cd Bascorro_ws`
3. `git clone https://github.com/EWS-Bascorro/src.git`
4. `cd ..`
5. `catkin_make`
6. `gedit ~/.basrhc`

7. in the bottom row add : \
        ```
        source /opt/ros/[ros distribution]/setup.bash
        ``` \
        ```
        source /home/[user]/Bascorro_ws/devel/setup.bash
        ```
8. save and exit
