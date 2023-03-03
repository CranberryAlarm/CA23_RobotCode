package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class Autonomous {
    private Trajectory limelightTrajectory;
    private ArrayList<Pose2d> waypoints;

    private double velocity = 1;
    private double acceleration = 0.25;

    public Autonomous() {
        this.limelightTrajectory = new Trajectory();
        waypoints = new ArrayList<Pose2d>();
    }

    /**
     * Adds a waypoint to the robot's trajectory (relative to the previous waypoint)
     * 
     * @param x Amount the robot should move in the X direction (in meters)
     * @param y Amount the robot should move in the Y direction (in meters)
     * @param rotation How much the robot should turn (in degrees)
     */
    public void addWaypoint(double x, double y, double rotation) {
        Pose2d previous = waypoints.get(waypoints.size()-1);
        Pose2d pose = new Pose2d(previous.getX() + x, previous.getY() + y, new Rotation2d(previous.getRotation().getDegrees() + rotation));
        Pose2d converted = toFieldPose(pose);
        waypoints.add(converted);
    }

    public void createTrajectory() {
        limelightTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, new TrajectoryConfig(velocity, acceleration));
    }

    public void runTrajectory(Drivetrain drive, RamseteController ramsete, double time) {
        Trajectory.State state = limelightTrajectory.sample(time);
        ChassisSpeeds speeds = ramsete.calculate(drive.getPose(), state);
        drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public void resetOdometry(double x, double y, double rotation, Drivetrain drive) {
        Pose2d reset = toFieldPose(new Pose2d(x, y, Rotation2d.fromDegrees(rotation)));
        drive.resetOdometry(reset);
        waypoints.add(reset);
    }

    /**
     * Converts the limelight coordinate system to the field coordinate system.
     * 
     * @param pose Position of the robot
     * @return The position of the robot in terms of the field.
     */
    private Pose2d toFieldPose(Pose2d pose) {
        return pose.relativeTo(new Pose2d(-8.2296, -8.2296 / 2, Rotation2d.fromDegrees(0)));
    }
}
