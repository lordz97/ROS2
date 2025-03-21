# ROS2Logging2024Group02

## Cloud storage link:
###   https://tucloud.tu-clausthal.de/index.php/s/AQo7QSsrM2xNYLA
###   password: 123456789



## Add your files

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://gitlab.tu-clausthal.de/isse/rg-dacs/studentische-arbeiten/ros2logging2024group02.git
git branch -M main
git push -uf origin main
```

## Integrate with your tools

- [ ] [Set up project integrations](https://gitlab.tu-clausthal.de/isse/rg-dacs/studentische-arbeiten/ros2logging2024group02/-/settings/integrations)

## Collaborate with your team

- [ ] [Invite team members and collaborators](https://docs.gitlab.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://docs.gitlab.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://docs.gitlab.com/ee/user/project/merge_requests/approvals/)
- [ ] [Set auto-merge](https://docs.gitlab.com/ee/user/project/merge_requests/merge_when_pipeline_succeeds.html)

## Test and Deploy

Use the built-in continuous integration in GitLab.

- [ ] [Get started with GitLab CI/CD](https://docs.gitlab.com/ee/ci/quick_start/index.html)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing (SAST)](https://docs.gitlab.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://docs.gitlab.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://docs.gitlab.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://docs.gitlab.com/ee/ci/environments/protected_environments.html)

***




## Name
Carla ROS2 Python project

## Description

This project integrates the Carla open-source simulator with the ROS2 middleware using Python. It has been designed to subscribe the lidar data published from a ROS2 node store them locally and then upload them in a cloud application.

## Precondition
have carla server installed , and the framework ros2 with the version "humble"

## run the carla server

In the carla directory, run the command "sh carlaUE4.sh" in a terminal

## how to run ros2 via the bridge using humble

go to the worskpace, open another terminal and run the following commands:
"source /opt/ros/humble/setup.bash" ;

"source install/setup.bash" .

then run the following command to launch ros2 client: "ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py"

# Run the program 

while ros2 is running enter the next command: "source install/setup.bash"
then run the subcriber : "ros2 run carla_data_manager lidar_subscriber"



## Authors and acknowledgment
Harisson Friedman ZEUFACK
Prosper Vieiry MBE TENE
Stella Raissa DJOMO NGUEYIM
Donald Elvis SADJOU
