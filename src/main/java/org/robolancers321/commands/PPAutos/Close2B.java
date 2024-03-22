/* (C) Robolancers 2024 */
package org.robolancers321.commands.PPAutos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.robolancers321.commands.AutoCommands.PathAndAutoPickup;
import org.robolancers321.commands.AutoCommands.PathAndIntake;
import org.robolancers321.commands.AutoCommands.PathAndMate;
import org.robolancers321.commands.AutoCommands.PathAndRetract;
import org.robolancers321.commands.AutoCommands.PathAndShoot;

import java.nio.file.Path;
import java.util.List;

import org.robolancers321.Constants.DrivetrainConstants;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class Close2B extends SequentialCommandGroup {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public Close2B() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Close2B");

    Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("Close2B");

    this.addCommands(
        // new InstantCommand(
        //     () -> this.drivetrain.setYaw(this.drivetrain.getPose().getRotation().getDegrees())),
        new InstantCommand(
          () -> this.drivetrain.resetPose(startingPose)),
        new PathAndShoot(pathGroup.get(0))
        
        );
  }
}
