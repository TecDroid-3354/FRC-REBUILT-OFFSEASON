package frc.tecdroid3354

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.tecdroid3354.constants.RobotDimensions
import frc.tecdroid3354.constants.RobotTelemetry
import frc.tecdroid3354.utils.degrees
import frc.tecdroid3354.utils.meters
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class RobotVisualizer(private val hoodPosition: Supplier<Angle>,
                      private val intakeDeployDisplacement: Supplier<Distance>) {
    //
    // VISUALIZATION IN 3D ONLY (requires 3D CAD asset configured in AdvantageScope)
    //

    /**
     * Updates the 2D and 3D (if applicable) visualizations of the robot through the supplied parameters
     * inside [RobotVisualizer]
     */
    fun updateRobotVisualization() {

        // Pivot pose at the fixed chassis offset with the live joint pitch angle
        val currentHoodPose = Pose3d( // TODO() = Correct orientation and axis of rotation of the hood
            Translation3d(), // Offset comes from CAD
            Rotation3d(0.0.degrees, hoodPosition.get().unaryMinus(), 0.0.degrees),
        )

        // Extend linearly along the local axis (Z-up or X-forward based on your CAD)
        val currentIntakePose = Pose3d( // TODO() = Account for angle at which the intake slides
            Translation3d(intakeDeployDisplacement.get(), 0.0.meters, 0.0.meters),
            Rotation3d()
        )

        // Logging results
        Logger.recordOutput(RobotTelemetry.SUBSYSTEM_VISUALIZATION_3D_TAB, currentIntakePose, currentHoodPose)
    }
}