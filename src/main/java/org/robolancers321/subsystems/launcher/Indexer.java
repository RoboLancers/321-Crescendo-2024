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

  private final DigitalInput entranceBeamBreak;
  private final DigitalInput exitBeamBreak;

  private double goalRPM = 0.0;

  private Indexer() {
    this.motor = new CANSparkFlex(IndexerConstants.kMotorPort, kBrushless);
    this.encoder = this.motor.getEncoder();
    this.controller = this.motor.getPIDController();

    this.entranceBeamBreak = new DigitalInput(IndexerConstants.kEntranceBeamBreakPort);
    this.exitBeamBreak = new DigitalInput(IndexerConstants.kExitBeamBreakPort);

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

  public boolean entranceBeamBroken() {
    return !this.entranceBeamBreak.get();
  }

  public boolean entranceBeamNotBroken() {
    return !this.entranceBeamBroken();
  }

  public boolean exitBeamBroken() {
    return !this.exitBeamBreak.get();
  }

  public boolean exitBeamNotBroken() {
    return !this.exitBeamBroken();
  }

  private void setRPM(double rpm) {
    this.controller.setReference(rpm, ControlType.kVelocity);
  }

  private void doSendables() {
    SmartDashboard.putNumber("indexer rpm", this.getRPM());
    SmartDashboard.putNumber("indexer goal rpm", this.goalRPM);

    SmartDashboard.putBoolean("indexer entrance beam broken", this.entranceBeamBroken());
    SmartDashboard.putBoolean("indexer exit beam broken", this.exitBeamBroken());
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

  public Command off() {
    return runOnce(
        () -> {
          this.goalRPM = 0.0;
        });
  }

  public Command shiftBackFromExit() {
    return runOnce(
            () -> {
              this.goalRPM = IndexerConstants.kShiftBackFromExitRPM;
            })
        .alongWith(new WaitUntilCommand(this::exitBeamNotBroken).withTimeout(1.0));
  }

  public Command shiftForwardToEntrance() {
    return runOnce(
            () -> {
              this.goalRPM = IndexerConstants.kShiftForwardFromEntranceRPM;
            })
        .alongWith(new WaitUntilCommand(this::entranceBeamBroken).withTimeout(1.0));
  }

  public Command shiftBackToEntrance() {
    return runOnce(
        () -> {
          this.goalRPM = IndexerConstants.kShiftBackToEntranceRPM;
        });
    // .alongWith(new WaitUntilCommand(this::exitBeamNotBroken).withTimeout(1.0));
  }

  public Command acceptHandoff() {
    return runOnce(
        () -> {
          this.goalRPM = IndexerConstants.kHandoffRPM;
        });
    // .alongWith(new WaitUntilCommand(this::exitBeamBroken).withTimeout(1.0));
  }

  public Command shiftFromHandoffForward() {
    return runOnce(
        () -> {
          this.goalRPM = IndexerConstants.kHandoffRPM;
        });
    // .alongWith(new WaitUntilCommand(this::entranceBeamNotBroken).withTimeout(1.0));
  }

  public Command outtake() {
    return this.runOnce(
            () -> {
              this.goalRPM = IndexerConstants.kOuttakeRPM;
            })
        .alongWith(
            new WaitUntilCommand(this::exitBeamBroken)
                .andThen(new WaitUntilCommand(this::exitBeamNotBroken))
                .andThen(new WaitCommand(0.1)))
        .withTimeout(1.0);
  }

  public Command revTrap() {
    return runOnce(
        () -> {
          this.goalRPM = IndexerConstants.kTrapRPM;
        });
  }

  public Command intakeSource() {
    return this.runOnce(
            () -> {
              this.goalRPM = IndexerConstants.kSourceRPM;
            })
        .alongWith(
            new WaitUntilCommand(() -> this.exitBeamBroken() && this.entranceBeamBroken())
                .withTimeout(0.5));
  }

  public Command tuneController() {
    this.initTuning();

    return run(this::tune);
  }
}
