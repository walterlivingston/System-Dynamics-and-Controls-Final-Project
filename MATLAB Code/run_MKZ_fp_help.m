%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	function [GPS,yaw_gyro,delta_enc, WP]=run_MKZ_fp(delta, Vel, X0, WP_FILE);
%
%	Assumes time step of Ts=0.01 (100 Hz).
%
%   Generates sensor measurement data for the MKZ given inital conditions 
%   (X0)at a specified velocity (Vel) and a commanded steer angle (delta).  
%   The function also returns a desired waypoint in (East,North) from a 
%   specified file (WP_FILE).
%
%	GPS=[East (m)  North(m)  Heading(rad)];
%   yaw_gryo=yaw_rate (rad/sec)
%   delta_enc=steering angle (rad)
%   WP = [East_desired (m)  North_desired (m)]
%
%   Vel = vehicle speed in m/s
%
%   X0=[East0 (m)  North0 (m)  Heading0 (rad) yaw_rate0 (rad/s)];
%   Default X0=[0 0 0 0];
%
%   WP_FILE (and corresponding X0)
%       0:  None (any, although a desored heading of 0, +/-90 deg, or +/-180 deg work well)
%       1:  Double Lane Change (0,0,pi,0)
%       2:  Track #1 (0,0,pi,0)
%       3:  Track #2, not available yet
%       4:  Track #3, not available yet
%
%   WP will return "NaN" for WP_FILE 0 and ALSO if the end of the waypoint
%   file is reached for any of the other Waypoint Files.  You can check
%   with the matlab functoin "isnan(WP)"
%   
%   Note: You must run "clear all" to reset the initial conditions.
%   You should also run "fclose('all'), but this is not critical.
%
%   Note: the max vehicle steer angle the vehicle can turn is 30 degrees
%
%   Note this code will also save the vehicles GPS postion to a file
%   titled "waypoint_file_for_GPSVisualizer.txt"  If you want to save each
%   run, you need to rename the file before executing the code again (as
%   each time the same file name will be over-written.  This file can then
%   be imported into GPS visualizer to create a KML file for importing into
%   Google Earth
%
%   Example:
%   [GPS(k+1,:),yaw_gyro(k+1),delta_enc(k+1)]=run_MKZ(10*pi/180, 30);
%   
%   The above will calculate the next GPS positions (E,N,Heading), Yaw
%   Gryo, and steer angle, 0.01 seconds after commanding 10 degrees of steer
%   angle at 30 m/s with all initial conditions set to zero.
%
%   Example:
%   [GPS(k+1,:),yaw_gyro(k+1),del_meas(k+1),WP(k+1,:)]=run_MKZ_fp(delta(k+1),Vel,X0,1);
%
%   The above will also return a desired waypoint for file #1
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%