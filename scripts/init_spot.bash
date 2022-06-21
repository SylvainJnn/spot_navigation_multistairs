#!/bin/bash

# call the 3 services to launch spot: claim, power_on, stand
rosservice call /spot/claim;

rosservice call /spot/power_on;

rosservice call /spot/stand;
