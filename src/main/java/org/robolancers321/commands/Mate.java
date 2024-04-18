/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.robolancers321.Constants.PivotConstants.PivotSetpoint;
import org.robolancers321.Constants.RetractorConstants.RetractorSetpoint;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class Mate extends SequentialCommandGroup {
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public Mate() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    SmartDashboard.putNumber("handoff retractor angle", RetractorSetpoint.kMating.angle);
    SmartDashboard.putNumber("handoff pivot angle", PivotSetpoint.kMating.angle);

    this.addCommands(
        this.flywheel.off(),
        this.indexer.off(),
        new ParallelCommandGroup(
            // supplier for tuning, use SD from above
            this.retractor.moveToAngle(RetractorSetpoint.kMating.angle),
            this.pivot.moveToAngle(PivotSetpoint.kMating.angle)),
        new ParallelCommandGroup(
                this.indexer.acceptHandoff(), this.sucker.out(), this.flywheel.acceptHandoff())
            .until(this.indexer::exitBeamBroken)
            .withTimeout(1.0),
        new WaitCommand(0.2),

        // this.indexer
        //     .acceptHandoff()
        //     .alongWith(this.sucker.out(), this.flywheel.acceptHandoff())
        //     .until(this.indexer::exitBeamBroken),

        // new ParallelDeadlineGroup(
        //     this.indexer.acceptHandoff(), this.sucker.out(), this.flywheel.acceptHandoff()),
        // new ParallelDeadlineGroup(
        //     this.indexer.shiftFromHandoffForward(),
        //     this.sucker.out(),
        //     this.flywheel.shiftForward()),

        this.sucker.offInstantly(),
        this.indexer.off(),
        this.flywheel.off(),
        new ParallelCommandGroup(this.retractor.moveToRetracted()));
    // this.pivot.moveToRetracted()));

    this.setName("Mate");
  }
}
