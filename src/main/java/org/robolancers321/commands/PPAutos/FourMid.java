/* (C) Robolancers 2024 */
package org.robolancers321.commands.PPAutos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;
import org.robolancers321.commands.AutoCommands.PathAndIntake;
import org.robolancers321.commands.AutoCommands.PathAndRetract;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class FourMid extends SequentialCommandGroup {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public FourMid() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("4Mid");

    this.addCommands(
        // TODO: test this
        new InstantCommand(
            () -> this.drivetrain.setYaw(this.drivetrain.getPose().getRotation().getDegrees())),
        new ScoreSpeakerFixedAuto(),
        new PathAndIntake(pathGroup.get(0)),
        new PathAndRetract(pathGroup.get(1)),
        new ScoreSpeakerFixedAuto(),
        new PathAndIntake(pathGroup.get(2)),
        new PathAndRetract(pathGroup.get(3)),
        new ScoreSpeakerFixedAuto(),
        new PathAndIntake(pathGroup.get(4)),
        new PathAndRetract(pathGroup.get(5)),
        new ScoreSpeakerFixedAuto()
        // new PathAndRetract(pathGroup.get(6)),
        // this.drivetrain.driveCommand(0, 0, 1.0, false)

        // new PathAndShoot(PathPlannerPath.fromPathFile("SweepStraight4M.5"))
        );
  }
}
