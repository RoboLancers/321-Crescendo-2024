/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.Constants.FlywheelConstants;
import org.robolancers321.Constants.PivotConstants;
import org.robolancers321.commands.AutoPickupNote;
import org.robolancers321.commands.EmergencyCancel;
import org.robolancers321.commands.IntakeNoteManual;
import org.robolancers321.commands.IntakeSource;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.OuttakeNote;
import org.robolancers321.commands.PPAutos.Close4B;
import org.robolancers321.commands.PPAutos.Close4T;
import org.robolancers321.commands.PPAutos.SweepStraight4M;
import org.robolancers321.commands.ScoreAmp;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.commands.ScoreSpeakerFixedTeleop;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.commands.ScoreTrap;
import org.robolancers321.commands.Shift;
import org.robolancers321.subsystems.Climber;
import org.robolancers321.subsystems.LED.LED;
import org.robolancers321.subsystems.LED.LED.Section;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.AimTable;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class RobotContainer {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;
  private Climber climber;

  private XboxController driverController;
  private XboxController manipulatorController;

  private SendableChooser<Command> autoChooser;

  private LED led;
  private AddressableLEDSim ledSim;

  private boolean climbing = false;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();
    this.climber = Climber.getInstance();

    this.driverController = new XboxController(0);
    this.manipulatorController = new XboxController(1);

    this.autoChooser = new SendableChooser<Command>();

    this.led = new LED();
    this.ledSim = new AddressableLEDSim(led.ledStrip);

    this.configureEvents();
    this.configureLEDs();
    this.configureDefaultCommands();
    this.configureDriverController();
    this.configureManipulatorController();
    this.configureAuto();
  }

  private void configureEvents() {
    RobotModeTriggers.autonomous().onTrue(disableClimbingMode());
    RobotModeTriggers.teleop().onTrue(disableClimbingMode());
  }

  private void configureLEDs() {
    // default, meteor red (not climbing)
    LED.registerSignal(0, () -> !climbing, LED.meteorRain(0.02, LED.kDrivingMeteor));

    // meteor blue (climbing)
    LED.registerSignal(1, () -> climbing, LED.meteorRain(0.02, LED.kClimbingMeteor));
    // sees note, blink orange
    LED.registerSignal(
        2, this.drivetrain::seesNote, LED.strobe(Section.FULL, new Color(180, 30, 0)));

    // note in sucker, solid white
    LED.registerSignal(3, this.sucker::noteDetected, LED.solid(Section.FULL, new Color(255, 255, 255)));

    // flywheel is revving, solid yellow
    LED.registerSignal(
        4,
        () ->
            (!this.flywheel.isRevved()
                && this.flywheel.getGoalRPM()
                    > FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm),
        LED.solid(Section.FULL, new Color(255, 255, 0)));

    // flywheel is revved, blink green
    LED.registerSignal(
        5,
        () ->
            (this.pivot.atGoal()
                && this.flywheel.isRevved()
                && this.flywheel.getGoalRPM()
                    > FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm),
        LED.strobe(Section.FULL, new Color(0, 255, 0)));
  }

  private void configureDefaultCommands() {
    this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(driverController, true));

    this.sucker.setDefaultCommand(this.sucker.off());
    this.indexer.setDefaultCommand(this.indexer.off());

    this.flywheel.setDefaultCommand(
        this.flywheel.revSpeakerFromRPM(
            () -> {
              if (this.indexer.entranceBeamNotBroken()) return 0.0;

              if (this.drivetrain.getDistanceToSpeaker() < 4.0)
                return Math.min(
                    0.7 * AimTable.interpolateFlywheelRPM(this.drivetrain.getDistanceToSpeaker()),
                    2000);

              return 0.0;
            }));

    this.retractor.setDefaultCommand(this.retractor.moveToRetracted());
    this.pivot.setDefaultCommand(
        this.pivot.aimAtSpeaker(
            () -> {
              if (this.indexer.entranceBeamNotBroken())
                return PivotConstants.PivotSetpoint.kRetracted.angle;

              if (this.drivetrain.getDistanceToSpeaker() < 4.0)
                return AimTable.interpolatePivotAngle(this.drivetrain.getDistanceToSpeaker());

              return PivotConstants.PivotSetpoint.kRetracted.angle;
            }));
  }

  /*
   * Press Bumpers Together: zero gyro
   *
   * Hold Right Bumper: toggle slow mode
   * Hold Left Bumper: intake
   *
   * Hold Right Trigger: deploy intake
   * Release Right Trigger: retract intake
   *
   * Hold Left Trigger: deploy outtake
   * Release Left Trigger: retract intake
   *
   * Press A Button: snap to amp angle
   * Press B Button: auto pickup note
   * Press X Button: emergency cancel
   * Press Y Button: enter/exit climb mode
   *
   */
  private void configureDriverController() {
    // TODO: i think we keep this just in case for teleop
    new Trigger(
            () -> this.driverController.getLeftBumper() && this.driverController.getRightBumper())
        .onTrue(this.drivetrain.zeroYaw());

    // TODO: technically we can just pull the bumper state off of the controller inside teleop
    // command but maybe this offers more control programmatically
    new Trigger(this.driverController::getRightBumper)
        .whileTrue(new InstantCommand(() -> this.drivetrain.slowMode = true));
    new Trigger(this.driverController::getRightBumper)
        .whileFalse(new InstantCommand(() -> this.drivetrain.slowMode = false));

    new Trigger(this.driverController::getLeftBumper).whileTrue(this.sucker.in());
    new Trigger(this.driverController::getLeftBumper).whileFalse(this.sucker.off());

    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
        .whileTrue(new IntakeNoteManual().unless(() -> climbing));

    // for auto handoff, putting it on a button instead is safer so fender shot is still viable
    // new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
    //     .onFalse((new Mate().andThen(new Shift())).onlyIf(this.sucker::noteDetected));

    new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
        .whileTrue(new OuttakeNote().unless(() -> climbing));

    new Trigger(this.driverController::getAButton).onTrue(
      // new IntakeSource());
      this.drivetrain.turnToAngle(90.0).unless(() -> climbing));
    new Trigger(this.driverController::getBButton).whileTrue(new AutoPickupNote().unless(() -> climbing));

    new Trigger(this.driverController::getXButton).onTrue(new EmergencyCancel().unless(() -> climbing));

    new Trigger(this.driverController::getYButton).onTrue(toggleClimbingMode());
  }

  /*
   * Press B Button: handoff
   *
   * Hold A Button: rev for score amp
   * Release A Button: eject for score amp
   *
   * Press Y Button: score speaker from distance
   *
   * Hold X Button: rev for fixed speaker shot
   * Release X Button: eject for fixed speaker shot
   *
   * Left Joystick Up: move shooter to amp
   * Left Joystick Down: move shooter to retracted
   *
   * CLIMB MODE
   *
   * Right Joystick Up: move both climbers up
   * Right Joystick down: move both climbers down
   * Right Bumper: right climber up
   * Right Trigger: right climber down
   * Left Bumper: left climber up
   * Left Trigger: left climber down
   */
  private void configureManipulatorController() {
    new Trigger(this.manipulatorController::getBButton).onTrue((new Mate().andThen(new Shift()).unless(() -> climbing)));

    // .onlyIf(this.sucker::noteDetected)

    new Trigger(this.manipulatorController::getAButton).whileTrue(new ScoreAmp().unless(() -> climbing));
    new Trigger(this.manipulatorController::getAButton)
        .onFalse(this.indexer.outtake().raceWith(Commands.idle(this.pivot, this.flywheel)).unless(() -> climbing));

    new Trigger(this.manipulatorController::getYButton).onTrue(new ScoreSpeakerFromDistance().unless(() -> climbing));

    new Trigger(this.manipulatorController::getXButton).and(() -> !climbing).whileTrue(new ScoreSpeakerFixedTeleop().unless(() -> climbing));
    new Trigger(this.manipulatorController::getXButton).and(() -> !climbing)
        .onFalse(
            new ParallelDeadlineGroup(
                this.indexer.outtake(),
                this.sucker.out(),
                Commands.idle(this.pivot, this.flywheel)).unless(() -> climbing));

    new Trigger(() -> this.manipulatorController.getLeftY() < -0.8).onTrue(this.pivot.aimAtAmp().unless(() -> climbing));
    new Trigger(() -> this.manipulatorController.getLeftY() > 0.8)
        .onTrue(this.pivot.moveToRetracted().unless(() -> climbing));

    new Trigger(() -> -this.manipulatorController.getRightY() > 0.2)
        .whileTrue(
            climber
                .run(
                    () -> {
                      climber.setLeftPower(-this.manipulatorController.getRightY());
                      climber.setRightPower(-this.manipulatorController.getRightY());
                    })
                .finallyDo(
                    () -> {
                      climber.setLeftPower(0);
                      climber.setRightPower(0);
                    })
                .onlyIf(() -> climbing));

    new Trigger(() -> -this.manipulatorController.getRightY() < -0.2)
        .whileTrue(
            climber
                .run(
                    () -> {
                      climber.setLeftPower(-this.manipulatorController.getRightY());
                      climber.setRightPower(-this.manipulatorController.getRightY());
                    })
                .finallyDo(
                    () -> {
                      climber.setLeftPower(0);
                      climber.setRightPower(0);
                    })
                .onlyIf(() -> climbing));

    new Trigger(this.manipulatorController::getLeftBumper)
        .whileTrue(
            climber
                .run(() -> climber.setLeftPower(0.2))
                .finallyDo(() -> climber.setLeftPower(0))
                .onlyIf(() -> climbing));

    new Trigger(this.manipulatorController::getRightBumper)
        .whileTrue(
            climber
                .run(() -> climber.setRightPower(0.2))
                .finallyDo(() -> climber.setRightPower(0))
                .onlyIf(() -> climbing));

    new Trigger(() -> this.manipulatorController.getLeftTriggerAxis() > 0.5)
        .whileTrue(
            climber
                .run(() -> climber.setLeftPower(-0.2))
                .finallyDo(() -> climber.setLeftPower(0))
                .onlyIf(() -> climbing));

    new Trigger(() -> this.manipulatorController.getRightTriggerAxis() > 0.5)
        .whileTrue(
            climber
                .run(() -> climber.setRightPower(-0.2))
                .finallyDo(() -> climber.setRightPower(0))
                .onlyIf(() -> climbing));

    // new Trigger(this.manipulatorController::getXButton).and(() -> climbing).onTrue(new ScoreTrap());
  }

  private void configureAuto() {
    /*
    Sweep - pickup front 3 notes with curves
    Sweep straight - pickup front 3 notes with straight line
    Close - pickup front note first
    Skip - pickup center note first then go back for front note
     */

    this.autoChooser.addOption("Do Nothing", Commands.idle(this.drivetrain, this.retractor, this.sucker, this.pivot, this.indexer, this.flywheel, this.climber));
    this.autoChooser.setDefaultOption("Score And Sit", new ScoreSpeakerFromDistance());

    // this.autoChooser.addOption("4NT Sweep", new Auto4NTSweep());
    // this.autoChooser.addOption("4NT Close", new Auto4NTClose());
    // this.autoChooser.addOption("3NT Close", new Auto3NTClose());

    // this.autoChooser.addOption("4NM Sweep", new Auto4NMSweep());
    // this.autoChooser.addOption("3NM Close", new Auto3NMClose());
    // this.autoChooser.addOption("4NM Sweep Fender", new Auto4NMSweepFender());
    // this.autoChooser.addOption("4NM Sweep Fender Straight", new Auto4NMSweepFenderStraight());
    // this.autoChooser.addOption(
    //     "4NM Sweep Fender Straight Auto Pickup", new Auto4NMSweepFenderStraightAutoPickup());

    // this.autoChooser.addOption("4NB Sweep", new Auto4NBSweep());
    // this.autoChooser.addOption("4NB Skip", new Auto4NBSkip());
    // this.autoChooser.addOption("3NB Sweep Straight", new Auto4NBSweepStraight());
    // this.autoChooser.addOption("3NB Close", new Auto3NBClose());

    // pathplanner
    // this.autoChooser.addOption("4 piece mid", new SweepStraight4M());
    this.autoChooser.addOption("score and taxi top", new Close4T());
    this.autoChooser.addOption("score and taxi bottom", new Close4B());
    // this.autoChooser.addOption("2 piece mid", new Close3M());

    SmartDashboard.putData(autoChooser);
  }

  private Command toggleClimbingMode() {
    return Commands.either(disableClimbingMode(), enableClimbingMode(), () -> climbing);
  }

  private Command enableClimbingMode() {
    return Commands.runOnce(
        () -> {
          this.climbing = true;
          this.pivot.setDefaultCommand(this.pivot.aimAtTrap());
        });
  }

  private Command disableClimbingMode() {
    return Commands.runOnce(
        () -> {
          this.climbing = false;
          this.pivot.setDefaultCommand(
              this.pivot.aimAtSpeaker(
                  () -> {
                    if (this.indexer.entranceBeamNotBroken())
                      return PivotConstants.PivotSetpoint.kRetracted.angle;

                    if (this.drivetrain.getDistanceToSpeaker() < 4.0)
                      return AimTable.interpolatePivotAngle(this.drivetrain.getDistanceToSpeaker());

                    return PivotConstants.PivotSetpoint.kRetracted.angle;
                  }));
        });
  }

  public Command getAutonomousCommand() {
    // return new Close4T();

    return this.autoChooser.getSelected();
  }
}
