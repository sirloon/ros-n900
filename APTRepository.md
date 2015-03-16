ros-n900 download files section can act as an APT repository. This is helpful to easily install ROS on N900 mobile phone. In order to do such, you'll need to install a new source from your N900.

  * login to your phone
  * edit `/etc/apt/sources.list` and add the following line:

```
deb http://ros-n900.googlecode.com/files /
```

  * run `apt-get update`
  * then install ROS, running `apt-get install ros-boxturtle-base`

(careful, this is a huge download, you should use a Wifi connection to preserve your dataplan).

`ros-boxturtle-base` package is optified, this means it will install in `/opt/ros-boxturtle-base/` directory, in `/opt` partition (2GB) instead of rootfs (256MB).

You can also install ROS tutorials with `apt-get install ros-boxturtle-tutorials`

Use the same URL to deal with Debian source packages:

```
deb-src http://ros-n900.googlecode.com/files /
```