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
import org.robolancers321.subsystems.launcher.AimTable.AimCharacteristic;

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

  private static final int kBeamBreakPort = 0;

  private static final double kDebounceTime = 0.05;

  /*
   * Implementation
   */

  public Pivot pivot;
  public Indexer indexer;
  public Flywheel flywheel;

  private DigitalInput beamBreak;
  private Debouncer beamBreakDebouncer;

  private AimTable aimTable;

  private Launcher() {
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.beamBreak = new DigitalInput(kBeamBreakPort);
    this.beamBreakDebouncer = new Debouncer(kDebounceTime, Debouncer.DebounceType.kBoth);

    this.aimTable = new AimTable();
  }

  public boolean getBeamBreakState() {
    return beamBreakDebouncer.calculate(beamBreak.get());
  }

  public Command acceptHandoff(){
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
      new RunCommand(() -> this.aimTable.updateSpeakerAimCharacteristic(distanceSupplier.getAsDouble())),
      new SequentialCommandGroup(
        indexer.shiftIntoPosition(this::getBeamBreakState),
        new ParallelRaceGroup(
            flywheel.yeetNoteSpeaker(() -> this.aimTable.getSpeakerAimCharacteristic().rpm),
            new SequentialCommandGroup(
                pivot.aimAtSpeaker(() -> this.aimTable.getSpeakerAimCharacteristic().angle),
                new WaitUntilCommand(flywheel::isRevved),
                indexer.outtake(this::getBeamBreakState),
                new WaitCommand(0.2))))
    );
  }
}
