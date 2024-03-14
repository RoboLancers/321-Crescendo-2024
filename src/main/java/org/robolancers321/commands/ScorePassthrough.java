/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.AimTable;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScorePassthrough extends SequentialCommandGroup {
  private Sucker sucker;

  private Indexer indexer;

  private Pivot pivot;
  private Flywheel flywheel;

  private Drivetrain drivetrain;
  private Retractor retractor;

  public ScorePassthrough() {

    this.sucker = Sucker.getInstance();
    this.indexer = Indexer.getInstance();
    this.pivot = Pivot.getInstance();
    this.flywheel = Flywheel.getInstance();
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();

    SmartDashboard.putBoolean("is manual pivot angle", false);
    SmartDashboard.putNumber("manual pivot angle", 10);
    SmartDashboard.putNumber("manual outtake rpm", 300);

    addCommands(
        new ParallelCommandGroup(
                this.drivetrain.turnToSpeaker(),
                this.pivot.aimAtSpeaker(
                    () -> AimTable.interpolatePivotAngle(this.drivetrain.getDistanceToSpeaker())),
                this.flywheel.revSpeakerFromRPM(
                    () -> AimTable.interpolateFlywheelRPM(this.drivetrain.getDistanceToSpeaker())),
                this.indexer.outtakeWithRPM(
                    () -> SmartDashboard.getNumber("manual outtake rpm", 300)),
                this.retractor
                    .moveToPassthrough(
                        () ->
                            SmartDashboard.getBoolean("is manual pivot angle", false)
                                ? SmartDashboard.getNumber("manual pivot angle", 10)
                                : SmartDashboard.getNumber("pivot goal position (deg)", 10))
                    .andThen(this.sucker.out()))
            .andThen(new ParallelCommandGroup(this.indexer.off(), this.flywheel.off()))
            .onlyIf(this.sucker::noteDetected));
  }
}
