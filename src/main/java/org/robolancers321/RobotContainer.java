/* (C) Robolancers 2024 */
package org.robolancers321;

import com.pathplanner.lib.auto.AutoBuilder;
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
    // works

    // this.drivetrain.resetPose(this.drivetrain.getPose());

    // var targ = new Pose2d(1, 5.3, Rotation2d.fromDegrees(0));

    var curr = drivetrain.getPose();

    // var ctrl1 = curr.plus(new Transform2d(-0.2, 0, Rotation2d.fromDegrees(0)));

    // var ctrl2 = curr.plus(new Transform2d(-0.8, 0, Rotation2d.fromDegrees(0)));

    // var end = curr.plus(new Transform2d(-1.0, 0, Rotation2d.fromDegrees(0)));

    // List<Translation2d> waypoints = PathPlannerPath.bezierFromPoses(curr, ctrl1, ctrl2, end);

    var path =
        // new PathPlannerPath(
        //     PathPlannerPath.bezierFromPoses(
        //         curr,
        //         curr.plus(new Transform2d(0.2, 0, Rotation2d.fromDegrees(0))),
        //         curr.plus(new Transform2d(0.8, 0, Rotation2d.fromDegrees(0))),
        //         curr.plus(new Transform2d(1, 0, Rotation2d.fromDegrees(0)))),
        //     Drivetrain.kAutoConstraints,
        //     new GoalEndState(0, Rotation2d.fromDegrees(90)));

        new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(
                curr,
                new Pose2d(1.2, 5.3, Rotation2d.fromDegrees(0)),
                new Pose2d(1, 5.3, Rotation2d.fromDegrees(0))),
            Drivetrain.kAutoConstraints,
            new GoalEndState(0, Rotation2d.fromDegrees(0)));

    // PathPlannerPath.fromPathPoints(
    //     List.<PathPoint>of(
    //         new PathPoint(new Translation2d(curr.getX(), curr.getY())),
    //         new PathPoint(new Translation2d(1, 5.3))),
    //     Drivetrain.kAutoConstraints,
    //     new GoalEndState(0, Rotation2d.fromDegrees(0)));

    // var path =
    //     new PathPlannerPath(
    //         waypoints, Drivetrain.kAutoConstraints, new GoalEndState(0, new Rotation2d()));

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);

    // works
    // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"));

    // works
    // return new PathPlannerAuto("Example Auto");
  }
}
