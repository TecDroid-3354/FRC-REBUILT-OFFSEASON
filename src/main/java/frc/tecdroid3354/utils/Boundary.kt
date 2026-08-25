package frc.tecdroid3354.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.tecdroid3354.constants.RobotConstants
import org.littletonrobotics.junction.Logger

/**
 * Creates a virtual box, which is intended to use as a boundary.
 *
 * Use standard Blue-Alliance coordinates (measures will match the ones in field drawings).
 *
 * Measure your coordinates as seeing the boundary from above, with bottom-corners pointing towards
 * the Alliance Wall of the boundary's alliance (e.g, if [isRedHalf] == true, bottom corners point towards Red Alliance Wall)
 *
 * The boundary can be seen as a box inside AdvantageScope under Field/Boundaries/[boundaryName];
 * make sure to set is type "Trajectory" when displaying it in a 3D / 2D field.
 *
 * @param isRedHalf Refers to the position of the boundary in the field, NOT your alliance.
 */
class Boundary(private val boundaryName: String, private val isRedHalf: Boolean,
               private val topLeft: Translation2d, private val topRight: Translation2d,
               private val bottomLeft: Translation2d, private val bottomRight: Translation2d) {

    /** Declares each boundary side as a trajectory state list that closes itself */
    private val boundarySides: List<Trajectory.State> = listOf(
        Trajectory.State(0.0, 0.0, 0.0, Pose2d(topLeft, Rotation2d()), 0.0),
        Trajectory.State(0.0, 0.0, 0.0, Pose2d(topRight, Rotation2d()), 0.0),
        Trajectory.State(0.0, 0.0, 0.0, Pose2d(bottomRight, Rotation2d()), 0.0),
        Trajectory.State(0.0, 0.0, 0.0, Pose2d(bottomLeft, Rotation2d()), 0.0),
        Trajectory.State(0.0, 0.0, 0.0, Pose2d(topLeft, Rotation2d()), 0.0)
    )

    /** Initializes a trajectory matching the boundary sides */
    private val boundaryBox: Trajectory = Trajectory(boundarySides)

    /** Used strictly to log the [boundaryBox] */
    init {
        Logger.recordOutput("Field/Boundaries/$boundaryName", boundaryBox)
    }

    /**
     * Checks if the given [Translation2d] is within boundaries.
     */
    fun contains(translation: Translation2d): Boolean {
        if (isRedHalf) {
            return translation.measureX.lte(bottomRight.measureX)
                    && translation.measureY.lte(bottomRight.measureY)
                    && translation.measureX.gte(topLeft.measureX)
                    && translation.measureY.gte(topLeft.measureY)
        }

        return translation.measureX.gte(bottomRight.measureX)
                && translation.measureY.gte(bottomRight.measureY)
                && translation.measureX.lte(topLeft.measureX)
                && translation.measureY.lte(topLeft.measureY)
    }
}