/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;

public class Launcher extends SubsystemBase {
  /*
   * Singleton
   */

  private static Launcher instance = null;

  public static Launcher getInstance() {
    if (instance == null) instance = new Launcher();

    return instance;
  }

  /*
   * Constants
   */

  // TODO
  // private static final int kBeamBreakPort = 0;

  // TODO
  // private static final double kDebounceTime = 0.05;

  /*
   * Implementation
   */

  public final Pivot pivot;
  public final Indexer indexer;
  public final Flywheel flywheel;

  // TODO
  // private final DigitalInput beamBreak;

  // TODO
  // private final Debouncer beamBreakDebouncer;

  // TODO
  // private final AimTable aimTable;

  private Launcher() {
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    // TODO
    // this.beamBreak = new DigitalInput(kBeamBreakPort);

    // TODO
    // this.beamBreakDebouncer = new Debouncer(kDebounceTime, Debouncer.DebounceType.kBoth);

    // TODO
    // this.aimTable = new AimTable();
  }

  public Command tuneMechanisms(){
    return new ParallelCommandGroup(
      this.pivot.tuneControllers(),
      this.indexer.tuneController(),
      this.flywheel.tuneController()
    );
  }

  // TODO
  /*

  public boolean getBeamBreakState() {
    return true;
    // return beamBreakDebouncer.calculate(beamBreak.get()); // TODO
  }

  public Command acceptHandoff() {
    return this.indexer.acceptHandoff(this::getBeamBreakState);
  }

  public Command yeetAmp() {
    return new SequentialCommandGroup(
        pivot.aimAtAmp(),
        new ParallelCommandGroup(
            flywheel.yeetNoteAmp(),
            indexer.outtake(this::getBeamBreakState),
            new WaitCommand(0.2)));
  }

  public Command yeetSpeaker(DoubleSupplier distanceSupplier) {
    return new ParallelRaceGroup(
        new RunCommand(
            () -> this.aimTable.updateSpeakerAimCharacteristic(distanceSupplier.getAsDouble())),
        new SequentialCommandGroup(
            indexer.shiftIntoPosition(this::getBeamBreakState),
            new ParallelRaceGroup(
                flywheel.yeetNoteSpeaker(() -> this.aimTable.getSpeakerAimCharacteristic().rpm),
                new SequentialCommandGroup(
                    pivot.aimAtSpeaker(() -> this.aimTable.getSpeakerAimCharacteristic().angle),
                    new WaitUntilCommand(flywheel::isRevved),
                    indexer.outtake(this::getBeamBreakState),
                    new WaitCommand(0.2)))));
  }

  */
}
