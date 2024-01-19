/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static org.robolancers321.util.TunableSet.Tunable.tune;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.robolancers321.util.TunableSet;

public class Indexer extends SubsystemBase {
  private static Indexer INSTANCE;

  public static Indexer getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Indexer();
    }
    return INSTANCE;
  }

  /* Constants */
  public static final String tunablePrefix = "Tune Indexer";
  public boolean tuning = false;

  private final TunableSet tuner = new TunableSet("Indexer");
  private static int kDeviceID = 0;

  private static int kCurrentLimit = 0;

  private static double kWheelCircumference = 0;

  private static boolean kInverted = false;
  private static double kP = 0;

  private static double kI = 0;

  private static double kD = 0;

  private static double kFF = 0;

  private static int kBBC = 0;

  private static double kMinOutput = -1;

  private static double kMaxOutput = 1;

  private static double kMaxRPM = 5700;
  private double desiredVelocity;
  /*Indexer Motor*/
  private static CANSparkMax m_indexerMotor;

  private static SparkPIDController m_pidController;

  private static AbsoluteEncoder m_absoluteEncoder;

  private static DigitalInput bb = new DigitalInput(kBBC);

  private Indexer() {
    m_indexerMotor = new CANSparkMax(kDeviceID, kBrushless);

    configureMotor();

    m_pidController = m_indexerMotor.getPIDController();

    m_absoluteEncoder = m_indexerMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_absoluteEncoder.setVelocityConversionFactor(kWheelCircumference);
  }

  private void configureMotor() {
    m_indexerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    m_indexerMotor.setSmartCurrentLimit(kCurrentLimit);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    configureMotorValues();
  }

  private void configureMotorValues() {
    m_indexerMotor.setInverted(kInverted);
    m_absoluteEncoder.setInverted(kInverted);

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setFF(kFF);
  }

  private void setTunables() {
    // using static tune to avoid double prefix
    tuning = tune(tunablePrefix, tuning);

    if (tuning) {
      // Changing constant values, this does not matter as the constructor is run once with the
      // initial values
      kP = tuner.tune("kP", kP);
      kI = tuner.tune("kI", kI);
      kD = tuner.tune("kD", kD);
      kFF = tuner.tune("kFF", kFF);

      kInverted = tuner.tune("motor inverted", kInverted);

      // reset all the tunables

      configureMotorValues();
    }
  }

  @Override
  public void periodic() {
    setTunables();
    doSendables();
  }

  public void setDesiredVelocity(double desiredVelocity) {
    this.desiredVelocity = desiredVelocity;
  }

  public double getDesiredVelocity() {
    return this.desiredVelocity;
  }

  public void setSpeedRPM(double rpm) {
    m_indexerMotor.set(Math.signum(rpm) * (Math.abs(rpm) / kMaxRPM));
  }

  public void setSpeed(double speed) {

    m_indexerMotor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void intakeJawn() {
    m_pidController.setReference(this.desiredVelocity, CANSparkBase.ControlType.kVelocity);
  }
  // Probably not needed but nice to have
  public void outtakeJawn() {
    m_pidController.setReference(-this.desiredVelocity, CANSparkBase.ControlType.kVelocity);
  }

  public void stopSpinningJawn() {
    m_pidController.setReference(0.0, CANSparkBase.ControlType.kVelocity);
  }

  public boolean noteDetected() {
    return bb.get();
  }

  public double getIntakeVelocity() {
    return m_absoluteEncoder.getVelocity();
  }

  public Command manualIndex(double appliedSpeed) {
    return run(() -> setSpeed(appliedSpeed));
  }

  private void doSendables() {
    SmartDashboard.putNumber("Indexer velocity", getIntakeVelocity());
    SmartDashboard.putNumber("Indexer set speed", m_indexerMotor.get());
    SmartDashboard.putNumber("Indexer voltage", m_indexerMotor.getBusVoltage());
    SmartDashboard.putNumber("Indexer amperage", m_indexerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Indexer motor temp", m_indexerMotor.getMotorTemperature());
    SmartDashboard.putBoolean("Note detected", noteDetected());
  }
}
