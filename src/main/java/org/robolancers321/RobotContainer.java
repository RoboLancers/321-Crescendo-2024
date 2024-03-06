/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.commands.IntakeNote;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.OuttakeNote;
import org.robolancers321.commands.ScoreAmp;
import org.robolancers321.commands.ScoreSpeakerFixed;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.commands.autonomous.Auto3NBClose;
import org.robolancers321.commands.autonomous.Auto3NMClose;
import org.robolancers321.commands.autonomous.Auto3NTClose;
import org.robolancers321.commands.autonomous.Auto4NBSkip;
import org.robolancers321.commands.autonomous.Auto4NBSweep;
import org.robolancers321.commands.autonomous.Auto4NBSweepStraight;
import org.robolancers321.commands.autonomous.Auto4NMSweep;
import org.robolancers321.commands.autonomous.Auto4NTClose;
import org.robolancers321.commands.autonomous.Auto4NTSweep;
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
    // default
    LED.registerSignal(1, () -> true, LED.meteorRain(0.02, LED.kDefaultMeteorColors));

    // note in sucker
    LED.registerSignal(
        2, this.sucker::noteDetected, LED.solid(Section.FULL, new Color(180, 30, 0)));

    // flywheel is revving
    LED.registerSignal(
        3,
        () -> (!this.flywheel.isRevved() && this.flywheel.getGoalRPM() > 0),
        LED.solid(Section.FULL, new Color(255, 255, 255)));

    // flywheel is revved
    LED.registerSignal(
        4,
        () -> (this.flywheel.isRevved() && this.flywheel.getGoalRPM() > 0),
        LED.solid(Section.FULL, new Color(0, 255, 0)));
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
        .onFalse(new Mate().onlyIf(this.sucker::noteDetected));

    new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
        .whileTrue(new OuttakeNote());
    new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
        .onFalse(this.retractor.moveToRetracted());

    new Trigger(this.driverController::getAButton).onTrue(this.drivetrain.turnToAngle(0.0));
    // new Trigger(this.driverController::getXButton).whileTrue(this.drivetrain.turnToNote());
  }

  private void configureManipulatorController() {
    // TODO: do we give manipulator option to force mate?
    // new Trigger(this.manipulatorController::getBButton).onTrue(new Mate());

    new Trigger(this.manipulatorController::getAButton).onTrue(new ScoreAmp());
    new Trigger(this.manipulatorController::getYButton).onTrue(new ScoreSpeakerFromDistance());
    new Trigger(this.manipulatorController::getXButton).whileTrue(ScoreSpeakerFixed.rev());
    new Trigger(this.manipulatorController::getXButton).onFalse(ScoreSpeakerFixed.eject());
  }

  private void configureAuto() {
    this.autoChooser.setDefaultOption("Score And Sit", new ScoreSpeakerFixedAuto());
    this.autoChooser.addOption("4NT Close", new Auto4NTClose());
    this.autoChooser.addOption("3NT Sweep", new Auto4NTSweep());
    this.autoChooser.addOption("3NT Close", new Auto3NTClose());

    this.autoChooser.addOption("4NM Sweep", new Auto4NMSweep());
    this.autoChooser.addOption("3NM Close", new Auto3NMClose());

    this.autoChooser.addOption("4NB Skip", new Auto4NBSkip());
    this.autoChooser.addOption("4NB Sweep", new Auto4NBSweep());
    this.autoChooser.addOption("3NB Sweep Straight", new Auto4NBSweepStraight());
    this.autoChooser.addOption("3NB Close", new Auto3NBClose());
  }

  public Command getAutonomousCommand() {
    return this.autoChooser.getSelected();
  }
}
