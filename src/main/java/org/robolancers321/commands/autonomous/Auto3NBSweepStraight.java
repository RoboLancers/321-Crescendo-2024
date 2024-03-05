/* (C) Robolancers 2024 */
package org.robolancers321.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.commands.PathAndShoot;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

public class Auto3NBSweepStraight extends SequentialCommandGroup {
  private Drivetrain drivetrain;

  public Auto3NBSweepStraight() {
    this.drivetrain = Drivetrain.getInstance();

    this.addCommands(
        // TODO: test this
        new InstantCommand(
            () -> this.drivetrain.setYaw(this.drivetrain.getPose().getRotation().getDegrees())),
        new ScoreSpeakerFixedAuto(),
        new PathAndShoot("NoTeamBottomStraight.1"),
        new PathAndShoot("NoTeamBottomStraight.2"),
        new PathAndShoot("NoTeamBottomStraight.3"));
  }
}
