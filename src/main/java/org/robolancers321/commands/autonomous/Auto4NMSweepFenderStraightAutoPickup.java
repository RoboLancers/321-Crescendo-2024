package org.robolancers321.commands.autonomous;

import org.robolancers321.commands.AutoPickupNote;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.PathAndIntake;
import org.robolancers321.commands.PathAndRetract;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.commands.Shift;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto4NMSweepFenderStraightAutoPickup extends SequentialCommandGroup {
    private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public Auto4NMSweepFenderStraightAutoPickup() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.addCommands(
        // TODO: test this
        new InstantCommand(
            () -> this.drivetrain.setYaw(this.drivetrain.getPose().getRotation().getDegrees())),
        new ScoreSpeakerFromDistance().onlyIf(this.indexer::entranceBeamBroken),

        new PathAndIntake("4NM-SweepFenderStraightAutoPickup.1"),
        new AutoPickupNote(),
        AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraightAutoPickup.2"),
            new PathConstraints(3.0, 2.0, 3.0, 1.5))
        .alongWith((new Mate().andThen(new Shift())).onlyIf(this.sucker::noteDetected)),
        new ScoreSpeakerFromDistance().onlyIf(this.indexer::entranceBeamBroken),

        new PathAndIntake("4NM-SweepFenderStraightAutoPickup.3"),
        new AutoPickupNote(),
        AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraightAutoPickup.4"),
            new PathConstraints(3.0, 2.0, 3.0, 1.5))
        .alongWith((new Mate().andThen(new Shift())).onlyIf(this.sucker::noteDetected)),
        new ScoreSpeakerFromDistance().onlyIf(this.indexer::entranceBeamBroken),

        new PathAndIntake("4NM-SweepFenderStraightAutoPickup.5"),
        new AutoPickupNote(),
        AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraightAutoPickup.6"),
            new PathConstraints(3.0, 2.0, 3.0, 1.5))
        .alongWith((new Mate().andThen(new Shift())).onlyIf(this.sucker::noteDetected)),
        new ScoreSpeakerFromDistance().onlyIf(this.indexer::entranceBeamBroken)

        );
  }
}
