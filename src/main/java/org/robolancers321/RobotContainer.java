/* (C) Robolancers 2024 */
package org.robolancers321;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    // this.drivetrain.setDefaultCommand(this.drivetrain.tuneModules());
    // this.drivetrain.setDefaultCommand(this.drivetrain.dangerouslyRunTurn(0.05));
    this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(controller, true));

    new Trigger(this.controller::getAButton).onTrue(this.drivetrain.zeroYaw());
  }

  private void configureAutoChooser() {
    // NamedCommands.registerCommand("Say Hello", new PrintCommand("Hello"));

    // this.autoChooser.addOption("Do Nothing", new InstantCommand());
  }

  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("tuning");

    var curr = drivetrain.getPose();

    var path =
        new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(
                curr,
                new Pose2d(1.2, 5.3, Rotation2d.fromDegrees(0)),
                new Pose2d(1, 5.3, Rotation2d.fromDegrees(0))),
            Drivetrain.kAutoConstraints,
            new GoalEndState(0, Rotation2d.fromDegrees(0)));

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }
}
