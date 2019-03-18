more off;
clear all;
close all;

%the goal of this exercise is to find a calibration matrix X such that when we apply it to the odometry input (X*u), it would best
% match the scanned match odometry
%then by drawing the trajectory of the robot based on pure odometry, calibrated odometry and scanned matched odometry, we show that the result
% of odometry calibration is very good as it is very close to the scanned match odometry.


% add tools directory
addpath('tools')

% load the odometry measurements
load ../data/odom_motions

% the motions as they are estimated by scan-matching
load ../data/scanmatched_motions

% create our measurements vector z
z = [scanmatched_motions odom_motions];

% perform the calibration
X = ls_calibrate_odometry(z);
disp('calibration result'); disp(X);

% apply the estimated calibration parameters
calibrated_motions = apply_odometry_correction(X, odom_motions);

% compute the current odometry trajectory, the scanmatch result, and the calibrated odom
odom_trajectory = compute_trajectory(odom_motions);
scanmatch_trajectory = compute_trajectory(scanmatched_motions);
calibrated_trajectory = compute_trajectory(calibrated_motions);

% plot the trajectories
plot(
  odom_trajectory(:,1), odom_trajectory(:,2), ";Uncalibrated Odometry;",
  scanmatch_trajectory(:,1), scanmatch_trajectory(:,2), ";Scan-Matching;",
  calibrated_trajectory(:,1), calibrated_trajectory(:,2), ";Calibrated Odometry;");
print -dpng "../plots/odometry-calibration.png"
