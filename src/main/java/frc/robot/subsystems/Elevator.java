package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Elevator extends Subsystem {
  private static final double kPivotPowerOut = 0.40;
  private static final double kPivotPowerIn = -0.4;
  private static final double kExtensionPowerOut = 0.35;
  private static final double kExtensionPowerIn = -0.35;

  private static Elevator mInstance;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private SimulatableCANSparkMax mPivotMotor;
  private SimulatableCANSparkMax mExtensionMotor;

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  private Elevator() {
    mPivotMotor = new SimulatableCANSparkMax(Constants.kElevatorPivotMotorId, MotorType.kBrushless);
    mExtensionMotor = new SimulatableCANSparkMax(Constants.kElevatorExtensionMotorId, MotorType.kBrushless);

    mPivotMotor.setIdleMode(IdleMode.kBrake);
    mExtensionMotor.setIdleMode(IdleMode.kBrake);
    mExtensionMotor.setInverted(true);

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    double pivot_power = 0.0;
    double extension_power = 0.0;
  }

  public void extend() {
    mPeriodicIO.extension_power = kExtensionPowerOut;
  }

  public void retract() {
    mPeriodicIO.extension_power = kExtensionPowerIn;
  }

  public void raise() {
    mPeriodicIO.pivot_power = kPivotPowerIn;
  }

  public void lower() {
    mPeriodicIO.pivot_power = kPivotPowerOut;
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  @Override
  public void writePeriodicOutputs() {
    mPivotMotor.set(mPeriodicIO.pivot_power);
    mExtensionMotor.set(mPeriodicIO.extension_power);
  }

  @Override
  public void stop() {
    stopPivot();
    stopExtension();
  }

  public void stopPivot() {
    mPeriodicIO.pivot_power = 0.0;

    mPivotMotor.set(0.0);
  }

  public void stopExtension() {
    mPeriodicIO.extension_power = 0.0;

    mExtensionMotor.set(0.0);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Pivot motor power:", mPeriodicIO.pivot_power);
    SmartDashboard.putNumber("Extension motor power:", mPeriodicIO.extension_power);
  }
}
