FROM clydemcqueen/ardupilot:latest

# Copy parameters from orca4 project
WORKDIR /ardupilot
COPY orca_bringup/cfg/sub.parm .

# TODO not tested
ENTRYPOINT [ "/bin/bash", "-c", "ardupilot/build/sitl/bin/ardusub \
  -w -S -M JSON --sim-address=172.18.0.3 -I0 \
  --defaults sub.parm \
  --home 33.810313,-118.39386700000001,0.0,270.0" ]
