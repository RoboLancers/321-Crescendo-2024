/* (C) Robolancers 2024 */
package org.robolancers321.commands.launcher;

import static org.robolancers321.util.TunableSet.Tunable.tune;

import edu.wpi.first.wpilibj2.command.Command;
import org.robolancers321.subsystems.launcher.Indexer;

public class IndexNote extends Command {
  private final Indexer indexer = Indexer.getInstance();

  public IndexNote() {
    addRequirements(this.indexer);
  }

  @Override
  public void initialize() {
    var velocity = indexer.tuning ? tune("Desired Velocity", 0) : 0;
    indexer.setDesiredVelocity(velocity);
  }

  @Override
  public void execute() {
    indexer.intakeJawn();
  }

  @Override
  public boolean isFinished() {
    return indexer.noteDetected();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopSpinningJawn();
  }
}
