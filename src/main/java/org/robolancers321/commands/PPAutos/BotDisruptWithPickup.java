/* (C) Robolancers 2024 */
package org.robolancers321.commands.PPAutos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.robolancers321.commands.AutoPickupNote;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

public class BotDisruptWithPickup extends SequentialCommandGroup {
  private Drivetrain drivetrain;

  public BotDisruptWithPickup() {
    this.drivetrain = Drivetrain.getInstance();

    this.addCommands(
        new InstantCommand(
            () -> this.drivetrain.setYaw(this.drivetrain.getPose().getRotation().getDegrees())),
        new ScoreSpeakerFixedAuto(),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BotDisruptWithPickup")),
        new AutoPickupNote()
    );
  }
}
