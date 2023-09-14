package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

public class PoseEstimator {
  private AHRS m_gyro;

  private Limelight LL_R = new Limelight("LL_R");
  private Limelight LL_L = new Limelight("LL_R");
}
