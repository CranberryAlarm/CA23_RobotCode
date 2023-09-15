package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseEstimator {
  private AHRS m_gyro;

  private Drivetrain m_drive;
  private DifferentialDrivePoseEstimator m_poseEstimator;

  private Limelight LL_R;
  private Limelight LL_L;

  private PoseEstimator() {
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_drive.getKinematics(), m_gyro.getRotation2d(), 0, 0,
        new Pose2d(0, 0, new Rotation2d(0))); // TODO: Check if this is right

    m_gyro = new AHRS();

    m_drive = Drivetrain.getInstance();

    LL_R = new Limelight("LL_R");
    LL_L = new Limelight("LL_L");
  }

  public void resetOdometry(Pose2d pose) {
    m_drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_drive.getKinematics(), m_gyro.getRotation2d(),
        0, 0, new Pose2d(0, 0, new Rotation2d(0)));
    setPose(pose);
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
  }
}
