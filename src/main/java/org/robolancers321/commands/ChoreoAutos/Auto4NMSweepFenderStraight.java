/* (C) Robolancers 2024 */
package org.robolancers321.commands.ChoreoAutos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.commands.AutoCommands.PathAndIntake;
import org.robolancers321.commands.AutoCommands.PathAndMate;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class Auto4NMSweepFenderStraight extends SequentialCommandGroup {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public Auto4NMSweepFenderStraight() {
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
        // new ScoreSpeakerFixedAuto(),
        new ScoreSpeakerFromDistance().onlyIf(this.indexer::entranceBeamBroken),
        new PathAndIntake(PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraight.1")),
        new PathAndMate(PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraight.2")),
        // new ScoreSpeakerFixedAuto(),
        new ScoreSpeakerFromDistance().onlyIf(this.indexer::entranceBeamBroken),
        new PathAndIntake(PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraight.3")),
        new PathAndMate(PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraight.4")),
        // new ScoreSpeakerFixedAuto(),
        new ScoreSpeakerFromDistance().onlyIf(this.indexer::entranceBeamBroken),
        new PathAndIntake(PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraight.5")),
        new PathAndMate(PathPlannerPath.fromChoreoTrajectory("4NM-SweepFenderStraight.6")),
        // new ScoreSpeakerFixedAuto()
        new ScoreSpeakerFromDistance().onlyIf(this.indexer::entranceBeamBroken));
  }
}
