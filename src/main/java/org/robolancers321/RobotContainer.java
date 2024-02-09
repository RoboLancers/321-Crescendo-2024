/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

import org.robolancers321.subsystems.LED;
import org.robolancers321.subsystems.LED.LEDPattern;
import org.robolancers321.subsystems.LED.Section;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.robolancers321.commands.Mate;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.launcher.Launcher;
import org.robolancers321.subsystems.LED;
import org.robolancers321.subsystems.LED.LEDPattern;

public class RobotContainer {
  Drivetrain drivetrain;
  Intake intake;
  Launcher launcher;

  XboxController driverController;
  XboxController manipulatorController;

  // SendableChooser<Command> autoChooser;


  private LED led = LED.getInstance();
  private CommandXboxController controller = new CommandXboxController(0);
  LED led = new LED();
  AddressableLEDSim ledSim = new AddressableLEDSim(led.ledStrip);
  // private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();
    this.intake = Intake.getInstance();
    this.launcher = Launcher.getInstance();

    this.driverController = new XboxController(0);
    this.manipulatorController = new XboxController(1);

    // this.autoChooser = AutoBuilder.buildAutoChooser();

    this.configureBindings();
    // this.configureAutoChooser();
  }

 
  private void configureBindings() {

    LED.register(4, () -> controller.b().getAsBoolean(), LEDPattern.WAVE, Color.kBlue, Color.kRed); 
    LED.register(4, () -> controller.b().getAsBoolean(), LEDPattern.WAVE, Color.kBlue, Color.kRed);
    LED.register(3, () -> controller.y().getAsBoolean(), LEDPattern.SOLID, Color.kGreen, null);
    LED.register(
        2, () -> controller.x().getAsBoolean(), LEDPattern.BREATH, Color.kBlue, Color.kRed);
    LED.register(1, () -> controller.a().getAsBoolean(), LEDPattern.STROBE, Color.kYellow, null);
    // this.drivetrain.setDefaultCommand(this.drivetrain.tuneModules());

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
    // new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
    //     .onFalse(this.intake.retractor.moveToRetracted());

    new Trigger(this.driverController::getXButton).onTrue(this.launcher.scoreAmp());

    new Trigger(this.driverController::getAButton).onTrue(new Mate());

    // this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(driverController, true));

    // new Trigger(this.driverController::getAButton).onTrue(this.drivetrain.zeroYaw());

    // new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
    //     .onTrue(this.intake.retractor.moveToIntake());

    // new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
    //     .onFalse(this.intake.retractor.moveToRetracted());

    // new Trigger(() -> this.manipulatorController.getRightTriggerAxis() > 0.8)
    //     .onTrue(this.launcher.pivot.aimAtAmp());

    // new Trigger(() -> this.manipulatorController.getRightTriggerAxis() > 0.8)
    //     .onFalse(this.launcher.pivot.moveToRetracted());
  }

  // private void configureAutoChooser() {
  //   // NamedCommands.registerCommand("Say Hello", new PrintCommand("Hello"));

  //   // this.autoChooser.addOption("Do Nothing", new InstantCommand());
  // }

  public Command getAutonomousCommand() {
    return new InstantCommand();

    // return new PathPlannerAuto("tuning");

    // var curr = drivetrain.getPose();

    // var path =
    //     new PathPlannerPath(
    //         PathPlannerPath.bezierFromPoses(
    //             curr,
    //             new Pose2d(1.2, 5.3, Rotation2d.fromDegrees(0)),
    //             new Pose2d(1, 5.3, Rotation2d.fromDegrees(0))),
    //         Drivetrain.kAutoConstraints,
    //         new GoalEndState(0, Rotation2d.fromDegrees(0)));

    // path.preventFlipping = true;

    // return AutoBuilder.followPath(path);
  }
}
