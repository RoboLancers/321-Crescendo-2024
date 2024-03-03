/* (C) Robolancers 2024 */
package org.robolancers321;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.commands.DeployIntake;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.OuttakeNote;
import org.robolancers321.commands.ScoreAmp;
import org.robolancers321.commands.ScoreSpeakerFixed;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.commands.autonomous.Auto3NBSweep;
import org.robolancers321.commands.autonomous.Auto3NBSweepStraight;
import org.robolancers321.subsystems.LED;
import org.robolancers321.subsystems.LED.Section;
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

  // SendableChooser<Command> autoChooser;

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

    // this.autoChooser = AutoBuilder.buildAutoChooser();

    this.led = new LED();
    this.ledSim = new AddressableLEDSim(led.ledStrip);

    this.configureBindings();
    this.configureAuto();
  }

  private void configureBindings() {
    LED.registerSignal(1, () -> true, LED.meteorRain(0.02, LED.kDefaultMeteorColors));
    LED.registerSignal(2, this.indexer::jawnDetected, LED.solid(Section.FULL, new Color(180, 30, 0)));
    LED.registerSignal(3, () -> (!this.flywheel.isRevved() && this.flywheel.getGoalRPM() > 0), LED.solid(Section.FULL, new Color(255, 255, 255)));
    LED.registerSignal(4, () -> (this.flywheel.isRevved() && this.flywheel.getGoalRPM() > 0), LED.solid(Section.FULL, new Color(0, 255, 0)));

    // TODO: register led bindings here

    // this.drivetrain.setDefaultCommand(this.drivetrain.tuneController(driverController));

    // this.intake.retractor.setDefaultCommand(this.intake.retractor.tuneControllers());
    // this.intake.sucker.setDefaultCommand(this.intake.sucker.tuneController());

    // this.pivot.setDefaultCommand(this.pivot.tuneControllers());
    // this.launcher.indexer.setDefaultCommand(this.launcher.indexer.tuneController());
    // this.flywheel.setDefaultCommand(this.flywheel.tuneController());

    this.sucker.setDefaultCommand(this.sucker.off());
    this.indexer.setDefaultCommand(this.indexer.off());
    this.flywheel.setDefaultCommand(this.flywheel.off());

    // new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
    //     .whileTrue(new DeployIntake());
    // new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
    //     .onFalse(this.retractor.moveToRetracted());

    // new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
    //     .whileTrue(new OuttakeNote());
    // new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
    //     .onFalse(this.retractor.moveToRetracted());

    // new Trigger(this.manipulatorController::getBButton).onTrue(new Mate());
    // new Trigger(this.manipulatorController::getAButton).whileTrue(new ScoreAmp());
    // new Trigger(this.manipulatorController::getAButton)
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             new ParallelRaceGroup(this.indexer.acceptHandoff(), this.sucker.out()),
    //             new WaitCommand(0.5),
    //             this.flywheel.off(),
    //             new ParallelCommandGroup(
    //                 this.retractor.moveToRetracted(), this.pivot.moveToRetracted())));
    new Trigger(this.manipulatorController::getYButton).onTrue(new ScoreSpeakerFromDistance());
    // new Trigger(this.manipulatorController::getXButton).whileTrue(new ScoreSpeakerFixed());
    // new Trigger(this.manipulatorController::getXButton)
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             new ParallelRaceGroup(this.indexer.acceptHandoff(), this.sucker.out()),
    //             new WaitCommand(0.5),
    //             this.flywheel.off(),
    //             new ParallelCommandGroup(
    //                 this.retractor.moveToRetracted(), this.pivot.moveToRetracted())));

    // new Trigger(() -> this.manipulatorController.getLeftY() < -0.8).onTrue(this.pivot.aimAtAmp());
    // new Trigger(() -> this.manipulatorController.getLeftY() > 0.8)
    //     .onTrue(this.pivot.moveToRetracted());

    new Trigger(this.driverController::getAButton).onTrue(this.drivetrain.turnToAngle(0.0));
    new Trigger(this.driverController::getBButton).onTrue(this.drivetrain.turnToAngle(90.0));
    // new Trigger(this.driverController::getXButton).whileTrue(this.drivetrain.turnToNote());
    new Trigger(this.driverController::getYButton).onTrue(this.drivetrain.turnToSpeaker());

    this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(driverController, true));

    new Trigger(
            () -> this.driverController.getLeftBumper() && this.driverController.getRightBumper())
        .onTrue(this.drivetrain.zeroYaw());

    new Trigger(this.driverController::getRightBumper)
        .whileTrue(new InstantCommand(() -> this.drivetrain.slowMode = true));
    new Trigger(this.driverController::getRightBumper)
        .whileFalse(new InstantCommand(() -> this.drivetrain.slowMode = false));
  }

  private void configureAuto() {
    NamedCommands.registerCommand("Drivetrain Off", this.drivetrain.stop());

    NamedCommands.registerCommand("Score Speaker Fixed", new ScoreSpeakerFixedAuto());
    NamedCommands.registerCommand("Deploy Intake", new DeployIntake());
    NamedCommands.registerCommand("Retract Intake", this.retractor.moveToRetracted());
    NamedCommands.registerCommand(
        "Score From Distance", new Mate().andThen(new ScoreSpeakerFromDistance()));

    // this.autoChooser.addOption("Do Nothing", new InstantCommand());
  }

  public Command getAutonomousCommand() {
    // Command auto = new PathPlannerAuto("tuning");

    // // return this.drivetrain.zeroYaw().andThen(auto);
    // var path = PathPlannerPath.fromChoreoTrajectory("").getPreviewStartingHolonomicPose()
    // return AutoBuilder.buildAuto("3NB-Sweep");

    return new Auto3NBSweep();
    // return new Auto3NBSweepStraight();
  }
}
