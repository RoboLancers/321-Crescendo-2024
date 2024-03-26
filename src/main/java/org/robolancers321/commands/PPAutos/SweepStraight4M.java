/* (C) Robolancers 2024 */
package org.robolancers321.commands.PPAutos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.commands.AutoCommands.PathAndIntake;
import org.robolancers321.commands.AutoCommands.PathAndRetract;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class SweepStraight4M extends SequentialCommandGroup {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public SweepStraight4M() {
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
        new ScoreSpeakerFixedAuto(),
        new PathAndIntake(PathPlannerPath.fromPathFile("SweepStraight4M.1")),
        new PathAndRetract(PathPlannerPath.fromPathFile("SweepStraight4M.2")),
        new ScoreSpeakerFixedAuto(),
        new PathAndIntake(PathPlannerPath.fromPathFile("SweepStraight4M.3")),
        new PathAndRetract(PathPlannerPath.fromPathFile("SweepStraight4M.4")),
        new ScoreSpeakerFixedAuto(),
        new PathAndIntake(PathPlannerPath.fromPathFile("SweepStraight4M.5")),
        new PathAndRetract(PathPlannerPath.fromPathFile("SweepStraight4M.6")),
        new ScoreSpeakerFixedAuto()

        // new PathAndShoot(PathPlannerPath.fromPathFile("SweepStraight4M.5"))
        );
  }
}
