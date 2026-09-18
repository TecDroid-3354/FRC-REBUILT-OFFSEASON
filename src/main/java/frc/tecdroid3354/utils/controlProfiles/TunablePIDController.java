package frc.tecdroid3354.utils.controlProfiles;

import edu.wpi.first.math.controller.PIDController;

/** <p>Credits for this file: FRC Team 2910 2026CompetitionRobot-Public.<p/>
 * Link: https://github.com/FRCTeam2910/2026CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/util/TunablePIDController.java
 * */

/**
 * PIDController with gains that can be modified at runtime via the dashboard (tuning mode).
 *
 * <p>Extends WPILib PIDController to allow hot-swapping of PID gains without recompiling.
 * </p>
 */
@SuppressWarnings("unused")
public class TunablePIDController extends PIDController {
    private final LoggedTunableNumber tunableKP;
    private final LoggedTunableNumber tunableKI;
    private final LoggedTunableNumber tunableKD;

    /**
     * Construct a TunablePIDController with initial gains.
     *
     * @param name the name prefix used for dashboard entries
     * @param kp initial proportional gain
     * @param ki initial integral gain
     * @param kd initial derivative gain
     */
    public TunablePIDController(String name, double kp, double ki, double kd) {
        super(kp, ki, kd);

        tunableKP = new LoggedTunableNumber(name + "/KP", kp);
        tunableKI = new LoggedTunableNumber(name + "/KI", ki);
        tunableKD = new LoggedTunableNumber(name + "/KD", kd);
    }

    /**
     * Update gains from the dashboard if they have changed.
     *
     * <p>Should be called periodically (typically once per robot loop).
     * </p>
     */
    public void update() {
        if (tunableKP.hasChanged(hashCode())) {
            this.setP(tunableKP.get());
        }

        if (tunableKI.hasChanged(hashCode())) {
            this.setI(tunableKI.get());
        }

        if (tunableKD.hasChanged(hashCode())) {
            this.setD(tunableKD.get());
        }
    }
}
