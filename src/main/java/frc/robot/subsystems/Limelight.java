package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

public class Limelight extends Subsystem {
    private static Limelight m_limelight;
    private NetworkTable m_limelightTable;
    private String m_table;

    // TODO: Get Limelight offsets

    /**
     * Constructor
     */
    public Limelight(String table) {
        m_table = table;
        m_limelightTable = NetworkTableInstance.getDefault().getTable(m_table);
    }

    /**
     * Enable the LEDs
     */
    public void setLightEnabled(boolean enabled) {
        m_limelightTable.getEntry("ledMode").setNumber(enabled ? 3 : 1);
    }

    /**
     * Get the current bot position
     *
     * @return Current bot pose
     */
    public Pose2d getBotpose2D() {
        return toFieldPose(LimelightHelpers.getBotPose2d(m_table));
    }

    /**
     * Get whether there is a visible AprilTag
     *
     * @return If there is a visible AprilTag
     */
    public boolean seesAprilTag() {
        return m_limelightTable.getEntry("tv").getInteger(0) == 1;
    }

    public double getTimeOffset(double currentTime) {
        return currentTime - LimelightHelpers.getLatency_Pipeline(m_table);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public void outputTelemetry() {
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

    // // TODO: Check to see if this works
    // public static double[] getAveragePose(double[] limelightA, double[]
    // limelightB) {
    // double[] averagePose = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // for (int i = 0; i < limelightA.length; i++) {
    // averagePose[i] = (limelightA[i] + limelightB[i]) / 2.0;
    // }

    // return averagePose;
    // }
}