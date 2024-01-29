package org.robolancers321.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    /*
     * Singleton
     */

    private static Intake instance = null;

    public static Intake getInstance(){
        if (instance == null) instance = new Intake();

        return instance;
    }

    /*
     * Constants
     */

    // TODO: beam break port and any other resources/information that interops with retractor and sucker

    /*
     * Implementation
     */

    public Retractor retractor;
    public Sucker sucker;
    // TODO: beam break

    private Intake(){
        this.retractor = Retractor.getInstance();
        this.sucker = Sucker.getInstance();
        // TODO: beam break
    }

    // TODO: any getters, setters, and interop commands
}
