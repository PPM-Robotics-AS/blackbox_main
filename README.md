# Introduction

BLACKBOX is an automated trigger-based reporting, data recording and playback unit, collecting data from robot and manufacturing systems. It takes error reporting and recovery of industrial robot systems to a new level, by developing and utilizing the innovative ROS based framework. The framework is built upon components from the project partner’s previous research and existing ROS modules. 

The project duration is 8 months and the overall budget is 245kEUR, with 70kEUR support from ROSIN and the remaining 175k EUR covered by PPM Robotics AS. Milestone 1 provides  solutions to demonstrate the connectivity and portability of BLACKBOX. It is be possible to record and replay data from various devices and control the system with a user-friendly interface. The system offers automatic reporting and datastore.

Additionally, BLACKBOX will provide interfaces to ROS, rosbag and FlexGui 4.0. To ensure an optimal use of the BLACKBOX framework, knowledge of the following components and areas is required:
- ROS and rosbag
- FlexGui 4.0
- Basic knowledge of setting up a remote ftp storage
 
## Packages

The complete BLACKBOX project built up from the following repositories:
- [blackbox_main](https://github.com/PPM-Robotics-AS/blackbox_main): Trigger monitor, cloud uploader and FlexGui 4.0 project.
- [blackbox_control](https://github.com/PPM-Robotics-AS/blackbox_control): rosbag control package to interface to capabilities of rosbag through rosservice operations.
- [blackbox_documentation](https://github.com/PPM-Robotics-AS/blackbox_documentation): Contains all of the documentation related to the project.

## Documentation

For user and installation manuals, please visit [blackbox_documentation](https://github.com/PPM-Robotics-AS/blackbox_documentation) repository.
- BLACKBOX main code documentation [html](https://github.com/PPM-Robotics-AS/blackbox_documentation/main)
- BLACKBOX control code documentation [html](https://github.com/PPM-Robotics-AS/blackbox_documentation/control)
- BLACKBOX user documentation and installation manual [pdf](https://github.com/PPM-Robotics-AS/blackbox_documentation/raw/master/BlackBox%20User%20Manual.pdf)
- FlexGui 4.0 user manual [pdf](https://github.com/PPM-Robotics-AS/blackbox_documentation/raw/master/FlexGui%20User%20Documentation.pdf)

## License

Licensed under the [Apache License, Version 2.0 (the "License")](http://www.apache.org/licenses/LICENSE-2.0);
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 