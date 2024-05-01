/* (C) Robolancers 2024 */
package org.robolancers321;

import static org.robolancers321.util.MathUtils.epsilonEquals;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.Constants.FlywheelConstants;
import org.robolancers321.Constants.PivotConstants;
import org.robolancers321.Constants.RetractorConstants;
import org.robolancers321.Constants.RetractorConstants.RetractorSetpoint;
import org.robolancers321.commands.AutoCommands.PathAndRetract;
import org.robolancers321.commands.IntakeNote;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.PPAutos.BotDisrupt;
import org.robolancers321.commands.PPAutos.BotTaxi;
import org.robolancers321.commands.PPAutos.FourBottom;
import org.robolancers321.commands.PPAutos.FourMid;
import org.robolancers321.commands.PPAutos.FourTop;
import org.robolancers321.commands.PPAutos.FourTopAlt;
import org.robolancers321.commands.PPAutos.ScoreAndSit;
import org.robolancers321.commands.PPAutos.ThreeBotCenter;
import org.robolancers321.commands.PPAutos.ThreeBotCenterAlt;
import org.robolancers321.commands.PPAutos.ThreeTopCenter;
import org.robolancers321.commands.PPAutos.TopDisrupt;
import org.robolancers321.commands.PPAutos.TopTaxi;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
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

  private CommandXboxController driverController;
  private CommandXboxController manipulatorController;

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

    this.driverController = new CommandXboxController(0);
    this.manipulatorController = new CommandXboxController(1);

    this.configureNamedCommands();

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

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> SmartDashboard.putString("Curr Command", command.getName()));
  }

  private void configureLEDs() {
    // default, meteor red (not climbing)
    LED.registerSignal(0, () -> !climbing, LED.meteorRain(0.02, LED.kDrivingMeteor));

    // meteor blue (climbing)
    // TODO: more technically correct solution is probably propagating robot mode into signal sets?
    LED.registerSignal(15, () -> climbing, LED.meteorRain(0.02, LED.kClimbingMeteor));

    // intakeDown, solid orange
    LED.registerSignal(
        2,
        () -> epsilonEquals(this.retractor.getPositionDeg(), RetractorSetpoint.kIntake.angle, 30),
        LED.solid(Section.FULL, new Color(255, 255, 255)));

    // note in sucker, solid white
    LED.registerSignal(
        3, this.sucker::noteDetected, LED.solid(Section.FULL, new Color(100, 150, 150)));

    // flywheel is revving, solid yellow
    LED.registerSignal(
        4,
        () ->
            (!this.flywheel.isRevved()
                && this.flywheel.getGoalRPM()
                    > FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm),
        LED.solid(Section.FULL, new Color(150, 255, 0)));

    // flywheel is revved, solid green
    LED.registerSignal(
        5,
        () ->
            (this.pivot.atGoal()
                && this.flywheel.isRevved()
                && this.flywheel.getGoalRPM()
                    > FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm),
        LED.solid(Section.FULL, new Color(0, 255, 0)));

    LED.registerSignal(
        6,
        () -> this.retractor.getGoal() == RetractorConstants.RetractorSetpoint.kAmp.angle,
        LED.solid(Section.FULL, new Color(20, 0, 50)));
  }

  private void configureDefaultCommands() {
    this.drivetrain.setDefaultCommand( // this.drivetrain.tuneModules());
        this.drivetrain.teleopDrive(driverController.getHID(), true));

    this.sucker.setDefaultCommand(this.sucker.off().repeatedly());
    this.indexer.setDefaultCommand(this.indexer.off());

    this.flywheel.setDefaultCommand(
        this.flywheel.revSpeakerFromRPM(
            () -> {
              if (this.indexer.entranceBeamNotBroken()) return 0.0;

              if (this.drivetrain.getDistanceToSpeaker() < 4.0)
                return Math.min(
                    0.85 * AimTable.interpolateFlywheelRPM(this.drivetrain.getDistanceToSpeaker()),
                    2000);

              return 0.0;
            }));

    this.retractor.setDefaultCommand(
        // this.retractor.tuneControllers()
        this.retractor.moveToSpeaker());

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
    this.driverController
        .leftBumper()
        .and(this.driverController.rightBumper())
        .onTrue(this.drivetrain.zeroYaw());

    // TODO: technically we can just pull the bumper state off of the controller inside teleop
    // command but maybe this offers more control programmatically
    this.driverController
        .rightBumper()
        .whileTrue(Commands.runOnce(() -> this.drivetrain.slowMode = true))
        .whileFalse(Commands.runOnce(() -> this.drivetrain.slowMode = false));

    this.driverController
        .leftBumper()
        .whileTrue(this.sucker.in())
        .whileFalse(this.sucker.off().repeatedly());

    this.driverController.rightTrigger(0.8).whileTrue(manuallyIntakeNote().unless(() -> climbing));

    // for auto handoff, putting it on a button instead is safer so fender shot is still viable
    // new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
    //     .onFalse((new Mate().andThen(new Shift())).onlyIf(this.sucker::noteDetected));

    this.driverController.leftTrigger(0.8).whileTrue(outtakeNote().unless(() -> climbing));

    this.driverController.a().whileTrue(this.drivetrain.alignToAmp().unless(() -> climbing));

    this.driverController.b().whileTrue(autoPickupNote().unless(() -> climbing));

    this.driverController
        .x()
        .onTrue(
            Commands.runOnce(() -> {}, drivetrain, retractor, sucker, indexer, flywheel, pivot)
                .unless(() -> climbing));

    this.driverController.y().onTrue(toggleClimbingMode());
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
   * Left Trigger hold: rev feeder
   * Left Trigger release: shoot feeder
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
    this.manipulatorController.b().onTrue((new Mate().andThen(new Shift()).unless(() -> climbing)));

    // .onlyIf(this.sucker::noteDetected)
    this.manipulatorController
        .a()
        .whileTrue(scoreAmp().unless(() -> climbing))
        .onFalse(
            this.indexer
                .outtake()
                .raceWith(Commands.idle(this.pivot, this.flywheel))
                .unless(() -> climbing));

    this.manipulatorController.y().onTrue(new ScoreSpeakerFromDistance().unless(() -> climbing));

    this.manipulatorController
        .x()
        .and(() -> !climbing)
        .whileTrue(scoreSpeakerFixed().unless(() -> climbing))
        .onFalse(
            Commands.sequence(
                    this.retractor.moveToSpeaker(),
                    Commands.deadline(
                        Commands.sequence(
                                Commands.waitUntil(this.indexer::exitBeamBroken),
                                Commands.waitUntil(this.indexer::exitBeamNotBroken),
                                Commands.waitSeconds(0.1))
                            .withTimeout(1.0),
                        this.indexer.outtake(),
                        this.sucker.out(),
                        Commands.idle(this.pivot, this.flywheel)))
                .unless(() -> climbing));

    this.manipulatorController.leftBumper().onTrue(toggleClimbingMode());

    // new Trigger(() -> this.manipulatorController.getLeftBumper())
    //     .and(() -> !climbing)
    //     .onTrue(new AutoScoreTrap());

    this.manipulatorController
        .povUp()
        .and(() -> !climbing)
        .whileTrue(feed().unless(() -> climbing))
        .onFalse(
            Commands.deadline(
                    Commands.sequence(
                            Commands.waitUntil(this.indexer::exitBeamBroken),
                            Commands.waitUntil(this.indexer::exitBeamNotBroken),
                            Commands.waitSeconds(0.1))
                        .withTimeout(1.0),
                    this.indexer.outtake(),
                    this.sucker.out(),
                    Commands.idle(this.pivot, this.flywheel))
                .unless(() -> climbing));

    this.manipulatorController
        .rightTrigger(0.5)
        .and(() -> !climbing)
        .onTrue(scoreAmpWithIntake().unless(() -> climbing));

    // new Trigger(() -> this.manipulatorController.getRightTriggerAxis() > 0.5)
    //     .and(() -> !climbing)
    //     .onFalse(
    //         new ParallelDeadlineGroup(
    //                 new WaitCommand(0.4), this.sucker.ampShot(), Commands.idle(this.retractor))
    //             .unless(() -> climbing));

    new Trigger(() -> this.manipulatorController.getLeftY() < -0.8)
        .onTrue(this.pivot.aimAtAmp().unless(() -> climbing));
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

    // new Trigger(this.manipulatorController::getLeftBumper)
    //     .and(() -> climbing)
    //     .whileTrue(
    //         climber
    //             .run(() -> climber.setLeftPower(0.2))
    //             .finallyDo(() -> climber.setLeftPower(0))
    //             .onlyIf(() -> climbing));

    // new Trigger(this.manipulatorController::getRightBumper)
    //     .whileTrue(
    //         climber
    //             .run(() -> climber.setRightPower(0.2))
    //             .finallyDo(() -> climber.setRightPower(0))
    //             .onlyIf(() -> climbing));

    this.manipulatorController
        .leftTrigger(0.5)
        .whileTrue(
            climber
                .run(() -> climber.setLeftPower(-0.2))
                .finallyDo(() -> climber.setLeftPower(0))
                .onlyIf(() -> climbing));

    this.manipulatorController
        .rightTrigger(0.5)
        .whileTrue(
            climber
                .run(() -> climber.setRightPower(-0.2))
                .finallyDo(() -> climber.setRightPower(0))
                .onlyIf(() -> climbing));

    // new Trigger(this.manipulatorController::getXButton).and(() -> climbing).onTrue(new
    // ScoreTrap());
  }

  private void configureAuto() {
    /*
    Sweep - pickup front 3 notes with curves
    Sweep straight - pickup front 3 notes with straight line
    Close - pickup front note first
    Skip - pickup center note first then go back for front note
     */

    this.autoChooser.addOption(
        "Do Nothing",
        Commands.runOnce(
            () -> this.drivetrain.setYaw(this.drivetrain.getPose().getRotation().getDegrees())));
    this.autoChooser.setDefaultOption("Score And Sit", new ScoreAndSit());

    this.autoChooser.addOption(
        "TESTING DONT USE",
        Commands.runOnce(
                () -> this.drivetrain.setYaw(this.drivetrain.getPose().getRotation().getDegrees()))
            .andThen(new PathAndRetract(PathPlannerPath.fromPathFile("Bruh"))));

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
    this.autoChooser.addOption("4 piece mid", new FourMid());
    this.autoChooser.addOption("score and taxi top", new TopTaxi());
    this.autoChooser.addOption("score and taxi bottom", new BotTaxi());

    this.autoChooser.addOption("4 piece top", new FourTop());
    this.autoChooser.addOption("4 piece top second note", new FourTopAlt());

    this.autoChooser.addOption("3 piece bottom", new FourBottom());
    this.autoChooser.addOption("3 piece top center only", new ThreeTopCenter());

    this.autoChooser.addOption("3 piece bottom center only", new ThreeBotCenter());
    this.autoChooser.addOption("3 piece bottom center only second note", new ThreeBotCenterAlt());

    this.autoChooser.addOption("top chaos", new TopDisrupt());
    this.autoChooser.addOption("bottom chaos", new BotDisrupt());

    // this.autoChooser.addOption("2 piece mid", new Close3M());

    SmartDashboard.putData(autoChooser);
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("IntakeNote", new IntakeNote());
  }

  private Command toggleClimbingMode() {
    return Commands.either(disableClimbingMode(), enableClimbingMode(), () -> climbing);
  }

  private Command enableClimbingMode() {
    return Commands.runOnce(
        () -> {
          this.climbing = true;
          this.pivot.setDefaultCommand(this.pivot.moveToRetracted());
          // this.pivot.setDefaultCommand(this.pivot.aimAtTrap());
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

  private Command manuallyIntakeNote() {
    return this.retractor.moveToIntake().alongWith(this.sucker.in());
  }

  private Command outtakeNote() {
    return this.retractor.moveToOuttake().andThen(this.sucker.out());
  }

  private Command scoreAmp() {
    return Commands.sequence(this.pivot.aimAtAmp(), this.flywheel.revAmp(), Commands.idle());
  }

  private Command scoreAmpWithIntake() {
    return this.retractor
        .moveToAmp()
        .until(() -> this.retractor.atGoalTimed(0.5))
        .andThen(this.sucker.ampShot().withTimeout(0.4))
        .withTimeout(3.0);
  }

  private Command scoreSpeakerFixed() {
    return Commands.parallel(
        this.flywheel.revSpeaker(), this.pivot.aimAtSpeakerFixed(), Commands.idle());
  }

  private Command feed() {
    return Commands.parallel(
        this.flywheel.revFeeder(),
        this.retractor.moveToMating(),
        this.pivot.aimAtSpeakerFixed(),
        Commands.idle());
  }

  private Command autoPickupNote() {
    return this.sucker
        .in()
        .raceWith(
            Commands.sequence(
                this.retractor.moveToIntake(),
                Commands.waitSeconds(0.5),
                Commands.sequence(
                        this.drivetrain.driveIntoNote(),
                        this.drivetrain
                            .driveCommand(0.0, 1.5, 0.0, false)
                            .until(this.sucker::noteDetected)
                            .withTimeout(1.0),
                        this.drivetrain.stop(),
                        this.retractor.moveToMating())
                    .onlyIf(this.drivetrain::seesNote)))
        .andThen(this.sucker.off());
  }

  public Command getAutonomousCommand() {
    // return new Close4T();
    // return AutoBuilder.buildAuto("Tuning");

    return this.autoChooser.getSelected();
  }
}
