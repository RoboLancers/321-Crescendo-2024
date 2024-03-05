/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.robolancers321.Constants.IndexerConstants;

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
   * Implementation
   */

  private final CANSparkFlex motor;
  private final SparkPIDController controller;
  private final RelativeEncoder encoder;

  private final DigitalInput beamBreak;

  private double goalRPM = 0.0;

  private Indexer() {
    this.motor = new CANSparkFlex(IndexerConstants.kMotorPort, kBrushless);
    this.encoder = this.motor.getEncoder();
    this.controller = this.motor.getPIDController();

    this.beamBreak = new DigitalInput(IndexerConstants.kBeamBreakPort);

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(IndexerConstants.kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(IndexerConstants.kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  }

  private void configureEncoder() {
    this.encoder.setVelocityConversionFactor(1.0);
  }

  private void configureController() {
    this.controller.setP(0.0);
    this.controller.setI(0.0);
    this.controller.setD(0.0);
    this.controller.setFF(IndexerConstants.kFF);
  }

  public double getRPM() {
    return this.encoder.getVelocity();
  }

  public boolean jawnDetected() {
    return !this.beamBreak.get();
  }

  public boolean jawnNotDetected() {
    return !this.jawnDetected();
  }

  private void setRPM(double rpm) {
    this.controller.setReference(rpm, ControlType.kVelocity);
  }

  private void doSendables() {
    SmartDashboard.putNumber("indexer rpm", this.getRPM());
    SmartDashboard.putNumber("indexer goal rpm", this.goalRPM);
    SmartDashboard.putBoolean("indexer detected note", this.jawnDetected());
    SmartDashboard.putNumber("indexer voltage", this.motor.getBusVoltage());
  }

  @Override
  public void periodic() {
    this.setRPM(this.goalRPM);

    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber(
        "indexer kff", SmartDashboard.getNumber("indexer kff", IndexerConstants.kFF));
    SmartDashboard.putNumber("indexer target rpm", 0.0);
  }

  private void tune() {
    double tunedFF = SmartDashboard.getNumber("indexer kff", IndexerConstants.kFF);

    this.controller.setFF(tunedFF);

    double targetRPM = SmartDashboard.getNumber("indexer target rpm", 0.0);

    this.goalRPM = targetRPM;
  }

  private Command setGoalRPM(double rpm) {
    return runOnce(() -> this.goalRPM = rpm);
  }

  public Command off() {
    return setGoalRPM(0.0);
  }

  public Command acceptHandoff() {
    return setGoalRPM(IndexerConstants.kHandoffRPM)
        .alongWith(new WaitUntilCommand(this::jawnDetected))
        .withTimeout(1.0);
  }

  public Command shiftForward() {
    return setGoalRPM(IndexerConstants.kShiftForwardRPM)
        .alongWith(new WaitUntilCommand(this::jawnDetected))
        .andThen(this.off())
        .withTimeout(1.0);
  }

  public Command shiftBackward() {
    return setGoalRPM(IndexerConstants.kShiftBackwardRPM)
        .alongWith(new WaitUntilCommand(this::jawnNotDetected))
        .andThen(this.off())
        .withTimeout(1.0);
  }

  public Command shift(){
    return this.shiftForward().andThen(this.shiftBackward());
  }

  public Command outtake() {
    return setGoalRPM(IndexerConstants.kOuttakeRPM)
        .alongWith(new WaitUntilCommand(this::jawnNotDetected).andThen(new WaitCommand(0.5)))
        .andThen(this.off());
  }

  public Command tuneController() {
    this.initTuning();

    return run(this::tune);
  }
}
