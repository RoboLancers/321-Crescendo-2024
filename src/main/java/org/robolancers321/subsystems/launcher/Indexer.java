/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Indexer extends SubsystemBase {
  /*
   * Singleton
   */

  private static Indexer instance = null;

  public static Indexer getInstance() {
    if (instance == null) instance = new Indexer();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 0;
  private static final int kBeamBreakPort = 0;

  private static final boolean kInvertMotor = false;
  private static final int kCurrentLimit = 20;

  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kFF = 0;

  private static final double kMaxRPM = 5700;
  private static final double kIntakeSpeedRPM = 500;
  private static final double kOuttakeSpeedRPM = -500;

  /*
   * Implementation
   */

  private final CANSparkMax motor;
  private final SparkPIDController controller;
  private final RelativeEncoder encoder;

  private final DigitalInput beamBreak;

  private Indexer() {
    this.motor = new CANSparkMax(kMotorPort, kBrushless);
    this.encoder = this.motor.getEncoder();
    this.controller = this.motor.getPIDController();

    this.beamBreak = new DigitalInput(kBeamBreakPort);

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
  }

  private void configureMotor() {
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  }

  private void configureEncoder() {
    this.encoder.setInverted(kInvertMotor);
  }

  private void configureController() {
    this.controller.setP(kP);
    this.controller.setI(kI);
    this.controller.setD(kD);
    this.controller.setFF(kFF);
  }

  public double getIntakeVelocityRPM() {
    return this.encoder.getVelocity();
  }

  public boolean jawnDetected() {
    return this.beamBreak.get();
  }

  public void dangerouslySetRPM(double rpm) {
    this.motor.set(rpm / kMaxRPM);
  }

  public void dangerouslySetSpeed(double speed) {
    this.motor.set(speed);
  }

  public void intakeJawn() {
    this.controller.setReference(kIntakeSpeedRPM, ControlType.kVelocity);
  }

  public void outtakeJawn() {
    this.controller.setReference(kOuttakeSpeedRPM, ControlType.kVelocity);
  }

  public void stopSpinningJawn() {
    this.controller.setReference(0.0, ControlType.kVelocity);
  }

  public Command manualIndex(DoubleSupplier appliedSpeedSupplier) {
    return run(() -> dangerouslySetSpeed(appliedSpeedSupplier.getAsDouble()));
  }

  public Command manualIndex(double appliedSpeed) {
    return this.manualIndex(() -> appliedSpeed);
  }

  public Command index() {
    // TODO: until beam break goes from true to false, also maybe add a time delay
    // TODO: should this beam break be the same as shooter?
    return run(this::intakeJawn).until(this::jawnDetected).finallyDo(this::stopSpinningJawn);
  }

  private void doSendables() {
    SmartDashboard.putNumber("Indexer Velocity (rpm)", this.getIntakeVelocityRPM());
    SmartDashboard.putBoolean("Note Detected", this.jawnDetected());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  public void initTuning() {
    SmartDashboard.putNumber("indexer kp", SmartDashboard.getNumber("indexer kp", kP));
    SmartDashboard.putNumber("indexer kp", SmartDashboard.getNumber("indexer ki", kI));
    SmartDashboard.putNumber("indexer kp", SmartDashboard.getNumber("indexer kd", kD));
    SmartDashboard.putNumber("indexer kff", SmartDashboard.getNumber("indexer kff", kFF));
  }

  public void tune() {
    double tunedP = SmartDashboard.getNumber("indexer kp", kP);
    double tunedI = SmartDashboard.getNumber("indexer ki", kI);
    double tunedD = SmartDashboard.getNumber("indexer kd", kD);
    double tunedFF = SmartDashboard.getNumber("indexer kff", kFF);

    this.controller.setP(tunedP);
    this.controller.setI(tunedI);
    this.controller.setD(tunedD);
    this.controller.setFF(tunedFF);
  }
}
