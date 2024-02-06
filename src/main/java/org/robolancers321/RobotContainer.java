/* (C) Robolancers 2024 */
package org.robolancers321;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

public class RobotContainer {
  Vision vision = Vision.getInstance();

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

    new Trigger(this.controller::getBButton)
        .onTrue(
            Commands.runOnce(
                () -> {
                  final var poseEst = vision.getEstimatedGlobalPose().estimatedPose.toPose2d();

                  SmartDashboard.putNumber("pose estimate x", poseEst.getX());
                  SmartDashboard.putNumber("pose estimate y", poseEst.getY());
                  SmartDashboard.putNumber("pose heading", poseEst.getRotation().getDegrees());
                }));
  }

  private void configureAutoChooser() {
    // NamedCommands.registerCommand("Say Hello", new PrintCommand("Hello"));

    // this.autoChooser.addOption("Do Nothing", new InstantCommand());
  }

  public Command getAutonomousCommand() {
    // return new InstantCommand();

    this.drivetrain.resetPose(new Pose2d(2.0, 7.0, new Rotation2d()));

    List<Translation2d> waypoints =
        List.of(
            new Translation2d(2.0, 7.0),
            new Translation2d(2.5, 7.0),
            new Translation2d(3.5, 7.0),
            new Translation2d(4.0, 7.0));

    return AutoBuilder.followPath(
        new PathPlannerPath(
            waypoints, Drivetrain.kAutoConstraints, new GoalEndState(0, new Rotation2d())));

    // works
    // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"));

    // works
    // return new PathPlannerAuto("Example Auto");
  }
}
