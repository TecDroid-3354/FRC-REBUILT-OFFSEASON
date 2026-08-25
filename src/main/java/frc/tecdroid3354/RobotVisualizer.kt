package frc.tecdroid3354

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotDimensions
import frc.tecdroid3354.constants.RobotTelemetry
import frc.tecdroid3354.constants.RobotVisualization
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.utils.degrees
import frc.tecdroid3354.utils.meters
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d
import java.util.function.Supplier

class RobotVisualizer(private val jointPosition: Supplier<Angle>,
                      private val elevatorDisplacement: Supplier<Distance>) {
    //
    // VISUALIZATION IN 2D ONLY
    //
    private val robot2d: LoggedMechanism2d = LoggedMechanism2d(
        RobotVisualization.CANVAS_WIDTH.meters,
        RobotVisualization.CANVAS_HEIGHT.meters,
        RobotVisualization.CANVAS_COLOR,
    )
    private val armOrigin: LoggedMechanismRoot2d = robot2d.getRoot(
        RobotVisualization.MECHANISMS_ORIGIN_2D_NAME,
        RobotVisualization.CANVAS_WIDTH.div(2.0).meters,
        0.0, // Starts at the bottom of the canvas
    )
    private val armGuidingRail: LoggedMechanismLigament2d = armOrigin.append(
        LoggedMechanismLigament2d(
            RobotVisualization.MECHANISMS_ARM_GUIDING_RAIL_2D_NAME,
            RobotDimensions.ELEVATOR_MINIMUM_LENGTH.meters, // Initial length
            SubsystemsPresetTargets.JOINT_HOME_ANGLE.degrees, // Initial angle
            RobotVisualization.ARM_GUIDING_RAIL_WIDTH,
            RobotVisualization.GUIDING_RAIL_COLOR
        )
    )
    private val armDisplacementLigament: LoggedMechanismLigament2d = armGuidingRail.append(
        LoggedMechanismLigament2d(
            RobotVisualization.MECHANISMS_ARM_DISPLACEMENT_LIGAMENT_2D_NAME,
            (SubsystemsMovementLimits.ELEVATOR_DISPLACEMENT_LIMITS.minimum as Distance).meters, // Initial length
            RobotVisualization.ARM_DISPLACEMENT_LIGAMENT_INITIAL_ANGLE.degrees, // Relative to guiding rail
            RobotVisualization.ARM_DISPLACEMENT_LIGAMENT_WIDTH,
            RobotVisualization.ARM_DISPLACEMENT_LIGAMENT_COLOR
        )
    )
    private val armEndEffectorLigament: LoggedMechanismLigament2d = armDisplacementLigament.append(
        LoggedMechanismLigament2d(
            RobotVisualization.MECHANISMS_ARM_END_EFFECTOR_2D_NAME,
            RobotVisualization.ARM_END_EFFECTOR_HEIGHT.meters,
            RobotVisualization.ARM_END_EFFECTOR_RELATIVE_ANGLE.degrees, // Relative to displacement ligament
            RobotVisualization.ARM_END_EFFECTOR_WIDTH,
            RobotVisualization.ARM_END_EFFECTOR_COLOR
        )
    )

    //
    // VISUALIZATION IN 3D ONLY (requires 3D CAD asset configured in AdvantageScope)
    //

    private val pivotOffset: Translation3d = Translation3d(
        RobotDimensions.JOINT_FORWARD_OFFSET,
        0.0.meters,
        RobotDimensions.JOINT_UPWARD_OFFSET,
    )

    /**
     * Updates the 2D and 3D (if applicable) visualizations of the robot through the supplied parameters
     * inside [RobotVisualizer]
     */
    fun updateRobotVisualization() {
        // 2D UPDATE ONLY
        armGuidingRail.setAngle(jointPosition.get().degrees)
        armDisplacementLigament.setLength(elevatorDisplacement.get().meters)

        // 3D UPDATE ONLY
        // NOTE: for Botzilla, the pivot is a bit weirdly configured. For Tutankabot it doesn't work at all (different subsystems).
        // This is just with the purpose of leaving a starting point for when you need to visualize a robot in its respective project.
        // Just don't visualize the 2D / 3D components in AdvantageScope to see the full (stationary) robot.

        // Pivot pose at the fixed chassis offset with the live joint pitch angle
        val currentPivotPose = Pose3d(
            Translation3d(),//pivotOffset,
            Rotation3d(0.0.degrees, jointPosition.get().unaryMinus(), 0.0.degrees)
        )

        // Extend linearly along the local axis (Z-up or X-forward based on your CAD)
        val calculatedArmPose = currentPivotPose.transformBy(
            Transform3d(
                Translation3d(elevatorDisplacement.get(), 0.0.meters, 0.0.meters),
                Rotation3d() // Pure translation along local arm orientation
            )
        )

        // Since this template does not contain the "wrist" subsystem, and that I don't want to fix this right now,
        // this is horribly wrong.
        val calculatedEndEffectorPose = calculatedArmPose

        // Logging results
        Logger.recordOutput(RobotTelemetry.SUBSYSTEM_VISUALIZATION_2D_TAB, robot2d)
        Logger.recordOutput(RobotTelemetry.SUBSYSTEM_VISUALIZATION_3D_TAB + "/Arm Pose", calculatedArmPose)
        Logger.recordOutput(RobotTelemetry.SUBSYSTEM_VISUALIZATION_3D_TAB + "/End Effector Pose", calculatedEndEffectorPose)
    }
}