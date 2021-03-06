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

echo_and_run cd "/home/sachin/Desktop/ws/forward_kinematics/ws/src/external/DynamixelSDK/ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sachin/Desktop/ws/forward_kinematics/ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sachin/Desktop/ws/forward_kinematics/ws/install/lib/python2.7/dist-packages:/home/sachin/Desktop/ws/forward_kinematics/ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sachin/Desktop/ws/forward_kinematics/ws/build" \
    "/usr/bin/python" \
    "/home/sachin/Desktop/ws/forward_kinematics/ws/src/external/DynamixelSDK/ros/setup.py" \
    build --build-base "/home/sachin/Desktop/ws/forward_kinematics/ws/build/external/DynamixelSDK/ros" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/sachin/Desktop/ws/forward_kinematics/ws/install" --install-scripts="/home/sachin/Desktop/ws/forward_kinematics/ws/install/bin"
