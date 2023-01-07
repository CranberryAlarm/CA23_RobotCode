package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Intake extends Subsystem {
  private static final double kIntakePower = 0.75;
  private static final double kLightIntakePower = 0.25;
  private static final double kExhaustPower = -0.75;
  private static final double kLightExhaustPower = -0.25;

  private static Intake mInstance;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  private SimulatableCANSparkMax mIntakeMaster;
  private DoubleSolenoid mIntakeSolenoid;
  private PeriodicIO mPeriodicIO = new PeriodicIO();

  private Intake() {
    mIntakeMaster = new SimulatableCANSparkMax(Constants.kIntakeMasterId, MotorType.kBrushless);
    mIntakeMaster.setInverted(true);

    mIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kIntakeSolenoidForwardId,
        Constants.kIntakeSolenoidReverseId);

    mPeriodicIO = new PeriodicIO();

    isStowed = true;
  }

  public enum WantedState {
    IDLE,
    INTAKE,
    EXHAUST,
    LIGHT_EXHAUST,
  }

  public enum SystemState {
    IDLE,
    INTAKING,
    EXHAUSTING,
    LIGHT_INTAKING,
    LIGHT_EXHAUSTING,
  }

  private SystemState mSystemState = SystemState.IDLE;
  private boolean isStowed;

  private static class PeriodicIO {
    double motor_power = 0.0;
  }

  public void deploy() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    isStowed = false;
  }

  public void stow() {
    stop();
    mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    isStowed = true;
  }

  public void setSystemState(SystemState systemState) {
    mSystemState = systemState;
  }

  public SystemState getSystemState() {
    return mSystemState;
  }

  @Override
  public void periodic() {
    switch (mSystemState) {
      case IDLE:
        stow();
        break;
      case INTAKING:
        deploy();
        mPeriodicIO.motor_power = kIntakePower;
        break;
      case EXHAUSTING:
        deploy();
        mPeriodicIO.motor_power = kExhaustPower;
        break;
      case LIGHT_INTAKING:
        deploy();
        mPeriodicIO.motor_power = kLightIntakePower;
        break;
      case LIGHT_EXHAUSTING:
        deploy();
        mPeriodicIO.motor_power = kLightExhaustPower;
        break;
      default:
        System.out.println("Unexpected intake system state: " + mSystemState);
        break;
    }

    // TODO: make this not suck (use registered subsystem loops)
    writePeriodicOutputs();
  }

  @Override
  public void writePeriodicOutputs() {
    mIntakeMaster.set(mPeriodicIO.motor_power);
  }

  @Override
  public void stop() {
    mPeriodicIO.motor_power = 0.0;
    mIntakeMaster.set(0.0);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Intake motor power:", mPeriodicIO.motor_power);
    SmartDashboard.putBoolean("Intake is stowed:", isStowed);
  }
}
