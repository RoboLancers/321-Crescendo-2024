/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static org.robolancers321.util.TunableSet.Tunable.tune;

import com.revrobotics.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import org.robolancers321.util.TunableSet;

public class Pivot extends ProfiledPIDSubsystem {

  private static class PivotFeedForward {
    private double kG;

    private PivotFeedForward(double kG) {
      this.kG = kG;
    }

    public void setkG(double kG) {
      this.kG = kG;
    }

    // setpoint is in degrees
    public double calculate(double setpoint) {
      return kG * Math.cos(setpoint);
    }
  }

  private static Pivot INSTANCE;

  public static Pivot getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Pivot();
    }
    return INSTANCE;
  }

  /* Constants */

  private static String tunablePrefix = "Tune Pivot";

  private static boolean tuning = false;

  private static TunableSet tuner = new TunableSet("Pivot");
  private static double kP = 0.0;

  private static double kI = 0.0;

  private static double kD = 0.0;

  private static double kG = 0.0;

  private static double kMaxAngle = 270;

  private static double kMinAngle = 0.0;
  private static final int kCurrentLimit = 0;

  private static final int kNominalVoltage = 12;

  private static final double kDegreesPerRot = 360;

  private static final double kTolerance = 0.0;

  private static int kDeviceID = 0;

  public enum PivotSetpoint {
    SPEAKER(0.0),
    AMP(0.0);

    private double angle;

    PivotSetpoint(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return this.angle;
    }
  }

  private static double kMaxVelocity = 0.0;

  private static double kMaxAcceleration = 0.0;

  private static PivotFeedForward pivotFF;

  private static CANSparkMax m_pivotMotor;

  private static AbsoluteEncoder m_absoluteEncoder;

  private Pivot() {
    // TODO: Determine real PID values needed and configure them here
    //       as well as the TrapezoidProfile 'maxVelocity' & 'maxAcceleration' constraints
    super(
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration)));

    pivotFF = new PivotFeedForward(kG);

    m_pivotMotor = new CANSparkMax(kDeviceID, CANSparkLowLevel.MotorType.kBrushless);

    m_absoluteEncoder = m_pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_absoluteEncoder.setPositionConversionFactor(kDegreesPerRot);

    configureMotor();
  }

  private void configureMotor() {
    m_pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    m_pivotMotor.setSmartCurrentLimit(kCurrentLimit);
    m_pivotMotor.enableVoltageCompensation(kNominalVoltage);
    m_pivotMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) kMaxAngle);
    m_pivotMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) kMinAngle);
    m_pivotMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    m_pivotMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

    super.m_controller.setTolerance(kTolerance);
    super.m_controller.enableContinuousInput(kMinAngle, kMinAngle);
  }

  // used for tuning
  private void configureMotorValues() {
    super.m_controller.setPID(kP, kI, kD);
    pivotFF.setkG(kG);
  }

  private void setTunables() {
    tuning = tune(tunablePrefix, tuning);

    if (tuning) {
      kP = tuner.tune("kP", kP);
      kI = tuner.tune("kI", kI);
      kD = tuner.tune("kD", kD);
      kG = tuner.tune("kG", kG);

      kMaxAcceleration = tuner.tune("kMaxAcceleration", kMaxAcceleration);
      kMaxVelocity = tuner.tune("kMaxVelocity", kMaxVelocity);

      configureMotorValues();
    }
  }

  public void setAngleGoal(PivotSetpoint goal) {
    super.setGoal(new TrapezoidProfile.State(goal.getAngle(), 0));
  }

  public void setAngleGoal(double goal) {
    super.setGoal(new TrapezoidProfile.State(goal, 0));
  }

  @Override
  public void periodic() {
    setTunables();
    doSendables();
  }

  @Override
  public double getMeasurement() {

    return m_absoluteEncoder.getPosition();
  }

  @Override
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {

    m_pivotMotor.setVoltage(output + pivotFF.calculate(setpoint.position));
  }

  public boolean atSetpoint() {
    return super.m_controller.atSetpoint();
  }

  public void doSendables() {
    SmartDashboard.putBoolean("Pivot at setpoint", atSetpoint());
    SmartDashboard.putNumber("Pivot position", getMeasurement());
    SmartDashboard.putNumber("Pivot voltage", m_pivotMotor.getBusVoltage());
    SmartDashboard.putNumber("Pivot amperage", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot motor temp", m_pivotMotor.getMotorTemperature());
  }
}
