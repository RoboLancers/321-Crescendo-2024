/* (C) Robolancers 2024 */
package org.robolancers321;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.commands.drivetrain.TeleopDrive;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.launcher.Launcher;

public class RobotContainer {
  Drivetrain drivetrain;
  Launcher launcher;

  XboxController driveController;
  XboxController manipulatorController;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();
    this.launcher = Launcher.getInstance();

    this.driveController = new XboxController(0);
    this.manipulatorController = new XboxController(1);

    this.autoChooser = AutoBuilder.buildAutoChooser();

    this.configureBindings();
    this.configureAutoChooser();
  }

  private void configureBindings() {
    this.drivetrain.setDefaultCommand(new TeleopDrive(this.driveController));
    this.launcher.setDefaultCommand(launcher.tuneSpeeds());

    new Trigger(this.driveController::getAButton)
        .onTrue(new InstantCommand(this.drivetrain::zeroYaw));

    // new Trigger(() -> this.manipulatorController.getRightY() > 0.8).whileTrue(launcher.yeet());
    // new Trigger(() -> this.manipulatorController.getRightY() <
    // -0.8).whileTrue(launcher.pullIn());

  }

  private void configureAutoChooser() {
    // NamedCommands.registerCommand("Say Hello", new PrintCommand("Hello"));

    // this.autoChooser.addOption("Do Nothing", new InstantCommand());
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();

    // return new PathPlannerAuto("Example Auto");

    // return this.autoChooser.getSelected();
  }
}
