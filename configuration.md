# Configuration
1. `mkdir Bascorro_ws`
2. `cd Bascorro_ws`
3. `mkdir src`
4. `cd src`
5. `git clone https://github.com/EWS-Bascorro/Bascorro_ws.git`
6. `cd Bascorro_ws`
7. `catkin_make`
8. `gedit ~/.basrhc`

9. in the botton add : \
        ```
        source /opt/ros/kinetic/setup.bash
        ``` \
        ```
        source /home/[user]/Bascorro_ws/devel/setup.bash
        ```
10. save and exit