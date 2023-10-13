package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseEstimator extends Subsystem {
  private AHRS m_gyro;

  private static PoseEstimator m_instance;

  private Drivetrain m_drive;
  private DifferentialDrivePoseEstimator m_poseEstimator;

  private PoseEstimator() {
    m_drive = Drivetrain.getInstance();

    m_gyro = new AHRS();

    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_drive.getKinematics(), m_gyro.getRotation2d(), 0, 0,
        new Pose2d(0, 0, new Rotation2d(0))); // TODO: Check if this is right
  }

  public static PoseEstimator getInstance() {
    if (m_instance == null) {
      m_instance = new PoseEstimator();
    }
    return m_instance;
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
  }

  public void addVisionMeasurement(Pose2d pose) {
    double current_time = Timer.getFPGATimestamp();

    m_poseEstimator.addVisionMeasurement(pose, current_time);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_drive.getKinematics(), m_gyro.getRotation2d(),
        0, 0, new Pose2d(0, 0, new Rotation2d(0)));
    setPose(pose);
  }

  @Override
  public void periodic() {
    double current_time = Timer.getFPGATimestamp();

    m_poseEstimator.updateWithTime(
        current_time,
        m_gyro.getRotation2d(),
        m_drive.getLeftLeader().getEncoder().getPosition(),
        m_drive.getRightLeader().getEncoder().getPosition());
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("PoseEstimator/X", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("PoseEstimator/Y", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("PoseEstimator/Rot", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

  }
}
