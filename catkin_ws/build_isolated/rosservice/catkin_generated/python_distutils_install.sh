#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ubuntu/Peach_Bot/catkin_ws/src/ros_comm/rosservice"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/opt/ros/kinetic/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/opt/ros/kinetic/lib/python2.7/dist-packages:/home/ubuntu/Peach_Bot/catkin_ws/build_isolated/rosservice/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/Peach_Bot/catkin_ws/build_isolated/rosservice" \
    "/usr/bin/python" \
    "/home/ubuntu/Peach_Bot/catkin_ws/src/ros_comm/rosservice/setup.py" \
    build --build-base "/home/ubuntu/Peach_Bot/catkin_ws/build_isolated/rosservice" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/opt/ros/kinetic" --install-scripts="/opt/ros/kinetic/bin"
