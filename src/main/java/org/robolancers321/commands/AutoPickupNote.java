/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class AutoPickupNote extends SequentialCommandGroup {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;

  public AutoPickupNote() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(
      this.retractor.moveToIntake(),
      new ParallelRaceGroup(
        this.sucker.in(),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          this.drivetrain.turnToNote(),
          this.drivetrain.driveCommand(0.0, 2.0, 0.0, false).until(() -> !this.drivetrain.seesNote()),
          this.drivetrain.driveCommand(0.0, 2.0, 0.0, false).until(this.sucker::noteDetected).withTimeout(1.0)
        )
      ),
      this.sucker.offInstantly()
    );
  }
}
