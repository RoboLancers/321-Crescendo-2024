/* (C) Robolancers 2024 */
package org.robolancers321.commands.ChoreoAutos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.commands.AutoPickupNote;
import org.robolancers321.commands.IntakeNote;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class Auto4NMSweepFenderStraightAutoPickup extends SequentialCommandGroup {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  private PathConstraints pathfinderConstraints =
      new PathConstraints(3.0, 2.0, 2 * Math.PI, Math.PI);
  // ideal(ish) points
  // private Pose2d start = new Pose2d(1.50, 5.59, Rotation2d.fromRadians(0));
  // private Pose2d bottomNote = new Pose2d(1.74, 5.08, Rotation2d.fromRadians(-0.71));
  // private Pose2d middleNote = new Pose2d(1.46, 5.54, Rotation2d.fromRadians(0));
  // private Pose2d topNote = new Pose2d(1.87, 6.06, Rotation2d.fromRadians(0.52));

  // in shop
  private Pose2d start = new Pose2d(1.40, 5.52, Rotation2d.fromDegrees(0));
  private Pose2d bottomNote = new Pose2d(1.5, 4.48, Rotation2d.fromDegrees(-16));
  private Pose2d middleNote = new Pose2d(1.64, 5.29, Rotation2d.fromDegrees(-8.6));
  private Pose2d topNote = new Pose2d(2.11, 5.7, Rotation2d.fromDegrees(7));

  // comp points (TODO)

  public Auto4NMSweepFenderStraightAutoPickup() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.pathfinderConstraints = new PathConstraints(2.0, 2.0, 2 * Math.PI, Math.PI);

    this.addCommands(
        new InstantCommand(
            () -> this.drivetrain.setYaw(this.drivetrain.getPose().getRotation().getDegrees())),
        new ScoreSpeakerFixedAuto(),
        AutoBuilder.pathfindToPoseFlipped(this.bottomNote, this.pathfinderConstraints, 1.5)
            .raceWith(new IntakeNote()),
        new AutoPickupNote(),
        AutoBuilder.pathfindToPoseFlipped(this.start, this.pathfinderConstraints)
            .alongWith(this.retractor.moveToMating())
            .alongWith(this.flywheel.revSpeaker()),
        new ScoreSpeakerFixedAuto().onlyIf(this.sucker::noteDetected),
        AutoBuilder.pathfindToPoseFlipped(this.middleNote, this.pathfinderConstraints, 1.5)
            .raceWith(new IntakeNote()),
        new AutoPickupNote(),
        AutoBuilder.pathfindToPoseFlipped(this.start, this.pathfinderConstraints)
            .alongWith(this.retractor.moveToMating())
            .alongWith(this.flywheel.revSpeaker()),
        new ScoreSpeakerFixedAuto().onlyIf(this.sucker::noteDetected),
        AutoBuilder.pathfindToPoseFlipped(this.topNote, this.pathfinderConstraints, 1.5)
            .raceWith(new IntakeNote()),
        new AutoPickupNote(),
        AutoBuilder.pathfindToPoseFlipped(this.start, this.pathfinderConstraints)
            .alongWith(this.retractor.moveToMating())
            .alongWith(this.flywheel.revSpeaker()),
        new ScoreSpeakerFixedAuto());
  }
}
