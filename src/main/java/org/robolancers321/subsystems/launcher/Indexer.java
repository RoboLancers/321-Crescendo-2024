/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;
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

  private static final double kHandoffRPM = 1500;
  private static final double kReindexRPM = 500;
  private static final double kOuttakeRPM = 5000;

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

  public double getRPM() {
    return this.encoder.getVelocity();
  }

  public boolean jawnDetected() {
    return this.beamBreak.get();
  }

  private void dangerouslySetSpeed(double speed) {
    this.motor.set(speed);
  }

  private void setRPM(double rpm) {
    this.controller.setReference(rpm, ControlType.kVelocity);
  }

  private void doSendables() {
    SmartDashboard.putNumber("indexer rpm", this.getRPM());
    SmartDashboard.putBoolean("note detected", this.jawnDetected());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("indexer kp", SmartDashboard.getNumber("indexer kp", kP));
    SmartDashboard.putNumber("indexer kp", SmartDashboard.getNumber("indexer ki", kI));
    SmartDashboard.putNumber("indexer kp", SmartDashboard.getNumber("indexer kd", kD));
    SmartDashboard.putNumber("indexer kff", SmartDashboard.getNumber("indexer kff", kFF));
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("indexer kp", kP);
    double tunedI = SmartDashboard.getNumber("indexer ki", kI);
    double tunedD = SmartDashboard.getNumber("indexer kd", kD);
    double tunedFF = SmartDashboard.getNumber("indexer kff", kFF);

    this.controller.setP(tunedP);
    this.controller.setI(tunedI);
    this.controller.setD(tunedD);
    this.controller.setFF(tunedFF);
  }

  public Command manualIndex(DoubleSupplier appliedSpeedSupplier) {
    return run(() -> dangerouslySetSpeed(appliedSpeedSupplier.getAsDouble()));
  }

  public Command manualIndex(double appliedSpeed) {
    return this.manualIndex(() -> appliedSpeed);
  }

  private Command off() {
    return runOnce(() -> this.setRPM(0.0));
  }

  public Command acceptHandoff(BooleanSupplier beamBreakStateSupplier) {
    return run(() -> this.setRPM(kHandoffRPM)).until(beamBreakStateSupplier).finallyDo(this::off);
  }

  // It's like cocking a gun, get your mind out of the gutter
  public Command cock(BooleanSupplier beamBreakStateSupplier) {
    /*
     * TODO
     *
     * sequential
     *    while beam is broken, setRPM(kReindexRPM)
     *    while beam is not broken, setRPM(-kReindexRPM)
     * finally do set rpm to 0
     *
     *
     * this should also have a timeout for safety
     *
     */

    return new SequentialCommandGroup(
            run(() -> this.setRPM(kReindexRPM)).until(beamBreakStateSupplier),
            new WaitCommand(0.1),
            run(() -> this.setRPM(-kReindexRPM))
                .until(() -> !beamBreakStateSupplier.getAsBoolean()),
            new WaitCommand(0.2))
        .finallyDo(this::off);
  }

  public Command feed(BooleanSupplier beamBreakStateSupplier) {
    return new ParallelRaceGroup(
            run(() -> this.setRPM(kOuttakeRPM)),
            new SequentialCommandGroup(
                new WaitUntilCommand(beamBreakStateSupplier),
                new WaitUntilCommand(() -> !beamBreakStateSupplier.getAsBoolean())),
            new WaitCommand(
                0.4) // just incase beam break fails, stop after some safe amount of time
            )
        .finallyDo(this::off);
  }

  public Command tuneController() {
    this.initTuning();

    return run(this::tune);
  }
}
