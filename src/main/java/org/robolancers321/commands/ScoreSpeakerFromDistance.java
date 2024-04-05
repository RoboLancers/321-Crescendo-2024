/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.launcher.AimTable;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScoreSpeakerFromDistance extends SequentialCommandGroup {
  private Pivot pivot;
  private Retractor retractor;
  private Indexer indexer;
  private Flywheel flywheel;
  private Drivetrain drivetrain;

  public ScoreSpeakerFromDistance() {
    this.pivot = Pivot.getInstance();
    // this.retractor = Retractor.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();
    this.drivetrain = Drivetrain.getInstance();

    // NOTE: actual usage of these are now inside of aim table
    SmartDashboard.putNumber("tuning pivot angle", 0);
    SmartDashboard.putNumber("tuning flywheel rpm", 0);

    this.addCommands(
        this.indexer.shiftBackFromExit().withTimeout(0.1),
        this.indexer.off(),
        this.drivetrain.turnToSpeaker(),
        this.drivetrain.stop(),
        new ParallelCommandGroup(
            this.pivot.aimAtSpeaker(
                () -> AimTable.interpolatePivotAngle(this.drivetrain.getDistanceToSpeaker())),
            this.flywheel.revSpeakerFromRPM(
                () -> AimTable.interpolateFlywheelRPM(this.drivetrain.getDistanceToSpeaker()))),
        // TODO: pivot does not move:
        // .withTimeout(1.5)
        this.indexer.outtake(),
        this.indexer.off(),
        this.flywheel.off());
  }
}
