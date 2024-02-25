/* (C) Robolancers 2024 */
package org.robolancers321;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.ScoreSpeakerFixed;
import org.robolancers321.commands.ScoreStage;
import org.robolancers321.subsystems.LED;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.launcher.Launcher;

public class RobotContainer {
  Drivetrain drivetrain;
  Intake intake;
  Launcher launcher;

  XboxController driverController;
  XboxController manipulatorController;

  // SendableChooser<Command> autoChooser;

  LED led = new LED();
  AddressableLEDSim ledSim = new AddressableLEDSim(led.ledStrip);

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();
    this.intake = Intake.getInstance();
    this.launcher = Launcher.getInstance();

    this.driverController = new XboxController(0);
    this.manipulatorController = new XboxController(1);

    // this.autoChooser = AutoBuilder.buildAutoChooser();

    this.configureBindings();
    this.configureAuto();
  }

  private void configureBindings() {
    // TODO: register led bindings here

    // this.drivetrain.setDefaultCommand(this.drivetrain.tuneController(driverController));

    // this.intake.retractor.setDefaultCommand(this.intake.retractor.tuneControllers());
    // this.intake.sucker.setDefaultCommand(this.intake.sucker.tuneController());

    // this.launcher.pivot.setDefaultCommand(this.launcher.pivot.tuneControllers());
    // this.launcher.indexer.setDefaultCommand(this.launcher.indexer.tuneController());
    // this.launcher.flywheel.setDefaultCommand(this.launcher.flywheel.tuneController());

    this.intake.sucker.setDefaultCommand(this.intake.sucker.off());
    this.launcher.indexer.setDefaultCommand(this.launcher.indexer.off());
    this.launcher.flywheel.setDefaultCommand(this.launcher.flywheel.off());

    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
        .whileTrue(this.intake.deployIntake());
    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
        .onFalse(this.intake.retractor.moveToRetracted());

    new Trigger(this.driverController::getRightBumper).whileTrue(this.intake.sucker.in());

    new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
        .whileTrue(this.intake.outtakeNote());
    new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
        .onFalse(this.intake.retractor.moveToRetracted());

    new Trigger(this.manipulatorController::getYButton).onTrue(new ScoreSpeakerFixed());
    new Trigger(this.manipulatorController::getXButton)
        .onTrue(new Mate().andThen(this.launcher.scoreAmp().raceWith(this.intake.sucker.off())));

    new Trigger(() -> this.manipulatorController.getLeftY() < -0.8)
        .onTrue(this.launcher.pivot.aimAtAmp());
    new Trigger(() -> this.manipulatorController.getLeftY() > 0.8)
        .onTrue(this.launcher.pivot.moveToRetracted());

    this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(driverController, true));

    new Trigger(
            () -> this.driverController.getLeftBumper() && this.driverController.getRightBumper())
        .onTrue(this.drivetrain.zeroYaw());
  }

  private void configureAuto() {
    NamedCommands.registerCommand("Drivetrain Off", this.drivetrain.stop());

    NamedCommands.registerCommand("Score Speaker Fixed", new ScoreSpeakerFixed());
    NamedCommands.registerCommand("Score Stage", new ScoreStage());
    // NamedCommands.registerCommand("Score Top", (. . .));

    NamedCommands.registerCommand("Deploy Intake", this.intake.deployIntake());
    NamedCommands.registerCommand("Retract Intake", this.intake.retractor.moveToRetracted());

    // this.autoChooser.addOption("Do Nothing", new InstantCommand());
  }

  public Command getAutonomousCommand() {
    // Command auto = new PathPlannerAuto("tuning");

    // return this.drivetrain.zeroYaw().andThen(auto);
    return AutoBuilder.buildAuto("3NB");
  }
}
