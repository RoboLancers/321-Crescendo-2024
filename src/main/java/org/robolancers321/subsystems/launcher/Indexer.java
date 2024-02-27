/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;

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

  private static final int kMotorPort = 16;
  // private static final int kBeamBreakPort = 0; // TODO

  private static final boolean kInvertMotor = true;
  private static final int kCurrentLimit = 60;

  private static final double kFF = 0.000153;

  private static final double kHandoffRPM = 3000;
  private static final double kReindexRPM = 3000;
  private static final double kOuttakeRPM = 3000;

  /*
   * Implementation
   */

  private final CANSparkFlex motor;
  private final SparkPIDController controller;
  private final RelativeEncoder encoder;

  // private final DigitalInput beamBreak; // TODO

  private Indexer() {
    this.motor = new CANSparkFlex(kMotorPort, kBrushless);
    this.encoder = this.motor.getEncoder();
    this.controller = this.motor.getPIDController();

    // this.beamBreak = new DigitalInput(kBeamBreakPort); // TODO

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  }

  private void configureEncoder() {
    this.encoder.setVelocityConversionFactor(1.0);
  }

  private void configureController() {
    this.controller.setP(0.0);
    this.controller.setI(0.0);
    this.controller.setD(0.0);
    this.controller.setFF(kFF);
  }

  public double getRPM() {
    return this.encoder.getVelocity();
  }

  public boolean jawnDetected() {
    return true;
    // return this.beamBreak.get(); // TODO
  }

  private void setRPM(double rpm) {
    this.controller.setReference(rpm, ControlType.kVelocity);
  }

  private void doSendables() {
    SmartDashboard.putNumber("indexer rpm", this.getRPM());
    SmartDashboard.putBoolean("indexer detected note", this.jawnDetected());
    SmartDashboard.putNumber("indexer voltage", this.motor.getBusVoltage());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("indexer kff", SmartDashboard.getNumber("indexer kff", kFF));
    SmartDashboard.putNumber("indexer target rpm", 0.0);
  }

  private void tune() {
    double tunedFF = SmartDashboard.getNumber("indexer kff", kFF);

    this.controller.setFF(tunedFF);

    double targetRPM = SmartDashboard.getNumber("indexer target rpm", 0.0);

    this.setRPM(targetRPM);
  }

  public Command off() {
    return run(() -> this.setRPM(0.0));
  }

  public Command acceptHandoff(BooleanSupplier beamBreakStateSupplier) {
    return run(() -> this.setRPM(kHandoffRPM))
        .until(beamBreakStateSupplier)
        .andThen(
            run(() -> this.setRPM(-0.1 * kHandoffRPM))
                .until(() -> !beamBreakStateSupplier.getAsBoolean()))
        .andThen(runOnce(() -> this.setRPM(0.0)));
  }

  public Command shiftIntoPosition(BooleanSupplier beamBreakStateSupplier) {
    return new SequentialCommandGroup(
        run(() -> this.setRPM(kReindexRPM)).until(beamBreakStateSupplier).withTimeout(0.4),
        run(() -> this.setRPM(-kReindexRPM))
            .until(() -> !beamBreakStateSupplier.getAsBoolean())
            .withTimeout(2.0));
  }

  public Command outtake(BooleanSupplier beamBreakStateSupplier) {
    return new ParallelRaceGroup(
        run(() -> this.setRPM(kOuttakeRPM)),
        new SequentialCommandGroup(
            new WaitUntilCommand(beamBreakStateSupplier),
            new WaitUntilCommand(() -> !beamBreakStateSupplier.getAsBoolean())));
  }

  public Command tuneController() {
    this.initTuning();

    return run(this::tune);
  }
}
