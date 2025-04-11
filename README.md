km-mavros 1.0
科马航空特制版mavos
是KM-ligo的组件之一
主要改动：
1、明确坐标系转换东北地（ned）到东北天（enu）的转换。从原本的航空坐标系转换到机器人和自动驾驶车辆的坐标系，处理完成后在反向回传。
2、使用px4的odom消息直接获取本地位姿、线速度、加速度和协方差。原来是分别从单独位置、姿态语句获取位姿，存在时间不同步问题，而且没有协方差数据。
3、使用px4的odom直接发布成mavros pose和odom数据，不需要而外计算和转换，减低计算资源消耗。

MAVLink extendable communication node for ROS.

- Since 2014-08-11 this repository contains several packages.
- Since 2014-11-02 hydro support separated from master to hydro-devel branch.
- Since 2015-03-04 all packages also dual licensed under terms of BSD license.
- Since 2015-08-10 all messages moved to mavros\_msgs package
- Since 2016-02-05 (v0.17) frame conversion changed again
- Since 2016-06-22 (pre v0.18) Indigo and Jade separated from master to indigo-devel branch.
- Since 2016-06-23 (0.18.0) support MAVLink 2.0 without signing.
- Since 2017-08-23 (0.20.0) [GeographicLib][geolib] and it's datasets are required. Used to convert AMSL (FCU) and WGS84 (ROS) altitudes.
- Since 2018-05-11 (0.25.0) support building master for Indigo and Jade stopped. Mainly because update of console-bridge package.
- Since 2018-05-14 (0.25.1) support for Indigo returned. We use compatibility layer for console-bridge.
- Since 2019-01-03 (0.28.0) support for Indigo by master not guaranteed. Consider update to more recent distro.
- 2020-01-01 version 1.0.0 released, please see [#1369][iss1369] for reasons and its purpose.
- 2021-05-28 version 2.0.0 released, it's the first alpha release for ROS2.
