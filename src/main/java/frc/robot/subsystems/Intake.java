package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

  private DoubleSolenoid mIntakeSolenoid;

  private Intake() {
    mIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kIntakeSolenoidForwardId,
        Constants.kIntakeSolenoidReverseId);
  }

  public void open() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void close() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void stop() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putBoolean("Intake state:", mIntakeSolenoid.get() == Value.kForward);
  }
}
