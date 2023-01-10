package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake extends Subsystem {
  private static Intake mInstance;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  private Solenoid mIntakeSolenoid;

  private Intake() {
    mIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kIntakeSolenoidForwardId);
  }

  public void open() {
    mIntakeSolenoid.set(false);
  }

  public void close() {
    mIntakeSolenoid.set(true);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void stop() {
    mIntakeSolenoid.set(false);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putBoolean("Intake state:", mIntakeSolenoid.get());
  }
}
