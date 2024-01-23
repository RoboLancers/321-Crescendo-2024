/* (C) Robolancers 2024 */
package org.robolancers321;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.commands.drivetrain.TeleopDrive;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

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
    this.drivetrain.setDefaultCommand(new TeleopDrive(this.controller));

    new Trigger(this.controller::getAButton).onTrue(new InstantCommand(this.drivetrain::zeroYaw));
  }

  private void configureAutoChooser() {
    NamedCommands.registerCommand("Say Hello", new PrintCommand("Hello"));

    this.autoChooser.addOption("Do Nothing", new InstantCommand());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");

    // return this.autoChooser.getSelected();
  }
}
