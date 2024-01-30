/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

  private static final int kBeamBreakChannelPort = 1;

  private static final double kDebounceTime = 0.05;
  // TODO: beam break(s)

  /*
   * Implementation
   */

  public Pivot pivot;
  public Indexer indexer;
  public Flywheel flywheel;

  public AimTable aimTable;

  public Debouncer beamBreakDebouncer = new Debouncer(kDebounceTime, Debouncer.DebounceType.kBoth);

  public DigitalInput beamBreak;

  // TODO: beam break(s)

  private Launcher() {
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();
    this.aimTable = new AimTable();
    this.beamBreak = new DigitalInput(kBeamBreakChannelPort);
  }

  public boolean getBeamBreakState() {
    return beamBreakDebouncer.calculate(beamBreak.get());
  }

  public Command yeetAmp() {
    return new SequentialCommandGroup(
        pivot.aimAtAmp(),
        new ParallelCommandGroup(
            flywheel.yeetNoteAmp(),
            indexer.feed(() -> !getBeamBreakState()),
            new WaitCommand(0.2)));
  }

  public Command yeetSpeaker(DoubleSupplier distanceSupplier) {
    AimCharacteristic aimCharacteristic =
        aimTable.getLastAimCharacteristic(distanceSupplier.getAsDouble());

    return new SequentialCommandGroup(
        indexer.cock(this::getBeamBreakState),
        new ParallelRaceGroup(
            flywheel.yeetNoteSpeaker(() -> aimCharacteristic.rpm),
            new SequentialCommandGroup(
                pivot.aimAtSpeaker(() -> aimCharacteristic.angle),
                new WaitUntilCommand(flywheel::isRevved),
                new WaitCommand(0.1),
                indexer.feed(this::getBeamBreakState),
                new WaitCommand(0.2))));
  }
}
