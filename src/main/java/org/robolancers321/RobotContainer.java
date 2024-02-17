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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.launcher.Launcher;

public class RobotContainer {
  Drivetrain drivetrain;
  Intake intake;
  Launcher launcher;

  XboxController driverController;
  XboxController manipulatorController;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();
    this.intake = Intake.getInstance();
    this.launcher = Launcher.getInstance();

    this.driverController = new XboxController(0);
    this.manipulatorController = new XboxController(1);

    this.autoChooser = AutoBuilder.buildAutoChooser();

    this.configureBindings();
    this.configureAutoChooser();
  }

  private void configureBindings() {
    this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(driverController, true));

    new Trigger(this.driverController::getAButton).onTrue(this.drivetrain.zeroYaw());

    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8).onTrue(this.intake.retractor.moveToIntake());

    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8).onFalse(this.intake.retractor.moveToRetracted());

    new Trigger(() -> this.manipulatorController.getRightTriggerAxis() > 0.8).onTrue(this.launcher.pivot.aimAtAmp());

    new Trigger(() -> this.manipulatorController.getRightTriggerAxis() > 0.8).onFalse(this.launcher.pivot.moveToRetracted());
  }

  private void configureAutoChooser() {
    // NamedCommands.registerCommand("Say Hello", new PrintCommand("Hello"));

    // this.autoChooser.addOption("Do Nothing", new InstantCommand());
  }

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
