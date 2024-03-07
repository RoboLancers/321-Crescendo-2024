/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.Constants.FlywheelConstants;
import org.robolancers321.commands.AutoPickupNote;
import org.robolancers321.commands.EmergencyCancel;
import org.robolancers321.commands.IntakeNote;
import org.robolancers321.commands.OuttakeNote;
import org.robolancers321.commands.ScoreAmp;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.commands.ScoreSpeakerFixedTeleop;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.commands.autonomous.Auto3NBSweep;
import org.robolancers321.commands.autonomous.Auto3NBSweepStraight;
import org.robolancers321.subsystems.LED.LED;
import org.robolancers321.subsystems.LED.LED.Section;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
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

  private XboxController driverController;
  private XboxController manipulatorController;

  private SendableChooser<Command> autoChooser;

  private LED led;
  private AddressableLEDSim ledSim;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.driverController = new XboxController(0);
    this.manipulatorController = new XboxController(1);

    this.autoChooser = new SendableChooser<Command>();

    this.led = new LED();
    this.ledSim = new AddressableLEDSim(led.ledStrip);

    this.configureLEDs();
    this.configureDefaultCommands();
    this.configureDriverController();
    this.configureManipulatorController();
    this.configureAuto();
  }

  private void configureLEDs() {
    // default, meteor red
    LED.registerSignal(1, () -> true, LED.meteorRain(0.02, LED.kDefaultMeteorColors));

    // sees note, blink orange
    LED.registerSignal(
        2, this.drivetrain::seesNote, LED.strobe(Section.FULL, new Color(180, 30, 0)));

    // note in sucker, solid orange
    LED.registerSignal(
        3, this.sucker::noteDetected, LED.solid(Section.FULL, new Color(180, 30, 0)));

    // flywheel is revving, solid white
    LED.registerSignal(
        4,
        () ->
            (!this.flywheel.isRevved()
                && this.flywheel.getGoalRPM()
                    > FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm),
        LED.solid(Section.FULL, new Color(255, 255, 255)));

    // flywheel is revved, blink green
    LED.registerSignal(
        5,
        () ->
            (this.flywheel.isRevved()
                && this.flywheel.getGoalRPM()
                    > FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm),
        LED.strobe(Section.FULL, new Color(0, 255, 0)));
  }

  private void configureDefaultCommands() {
    this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(driverController, true));

    this.sucker.setDefaultCommand(this.sucker.off());
    this.indexer.setDefaultCommand(this.indexer.off());
    this.flywheel.setDefaultCommand(this.flywheel.off());
  }

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

    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
        .whileTrue(new IntakeNote());
    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
        .onFalse(this.retractor.moveToRetracted());

    new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
        .whileTrue(new OuttakeNote());
    new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
        .onFalse(this.retractor.moveToRetracted());

    new Trigger(this.driverController::getAButton).onTrue(this.drivetrain.turnToAngle(90.0));
    new Trigger(this.driverController::getBButton).onTrue(new AutoPickupNote());
    new Trigger(this.driverController::getXButton).onTrue(new EmergencyCancel());
  }

  private void configureManipulatorController() {
    new Trigger(this.manipulatorController::getAButton)
        .onTrue(
            new ScoreAmp()
                .finallyDo(
                    () -> {
                      CommandScheduler.getInstance()
                          .schedule(
                              new ParallelCommandGroup(
                                  this.retractor.moveToRetracted(), this.pivot.moveToRetracted()));
                    }));
    new Trigger(this.manipulatorController::getYButton)
        .onTrue(
            new ScoreSpeakerFromDistance()
                .finallyDo(
                    () -> {
                      CommandScheduler.getInstance()
                          .schedule(
                              new ParallelCommandGroup(
                                  this.retractor.moveToRetracted(), this.pivot.moveToRetracted()));
                    }));
    new Trigger(this.manipulatorController::getXButton).whileTrue(new ScoreSpeakerFixedTeleop());
    new Trigger(this.manipulatorController::getXButton)
        .onFalse(
            this.indexer
                .outtake()
                .raceWith(this.sucker.out())
                .alongWith(new InstantCommand(() -> {}, this.flywheel)));
  }

  private void configureAuto() {
    this.autoChooser.setDefaultOption("Score And Sit", new ScoreSpeakerFixedAuto());
    this.autoChooser.addOption("3NB Sweep", new Auto3NBSweep());
    this.autoChooser.addOption("3NB Sweep Straight", new Auto3NBSweepStraight());
  }

  public Command getAutonomousCommand() {
    // return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("3NM"));

    return this.autoChooser.getSelected();
  }
}
