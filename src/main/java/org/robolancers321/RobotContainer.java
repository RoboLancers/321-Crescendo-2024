/* (C) Robolancers 2024 */
package org.robolancers321;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.drivetrain.SwerveModule;

public class RobotContainer {
  Drivetrain drivetrain;

  XboxController controller;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();
    this.controller = new XboxController(0);

    this.autoChooser = AutoBuilder.buildAutoChooser();

    this.configureBindings();
    this.configureAutoChooser();
  }

  private void configureBindings() {
    this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(controller, true));

    new Trigger(this.controller::getAButton).onTrue(this.drivetrain.zeroYaw());
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
