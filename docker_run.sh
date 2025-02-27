xhost +local:

docker run -it --network=host --ipc=host -v $PWD:/atwork_ws/ --env=DISPLAY --privileged rc_atwork $@

# the --privileged flag is used so that the serial ports are available in the
# container but it is not secure and should be changed in the future.