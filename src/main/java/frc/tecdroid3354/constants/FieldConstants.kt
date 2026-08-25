package frc.tecdroid3354.constants

import edu.wpi.first.math.geometry.Translation2d
import frc.tecdroid3354.utils.Boundary
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.meters

object FieldConstants {
    /**
     * Use it to store all field boundaries (think of them as box) that will have some effect
     * on robot functionality.
     *
     * For example, in 2026 REBUILT, if your driver was trying to shoot behind the Tower,
     * the robot should stop shooting to avoid FUELS colliding with tower bars.
     *
     * Get your measures from official field drawings and read the [Boundary] documentation on how to pass them.
     */
    object BoundaryLimits {
        val BLUE_TOWER_BOUNDARY         : Boundary = Boundary(
            "Blue_Tower", false,
            Translation2d((41.56 + 4.0).inches, (170.97 + 4.0).inches), Translation2d((41.56 + 4.0).inches, (123.97 - 4.0).inches),
            Translation2d(0.0.inches, (170.97 + 4.0).inches), Translation2d(0.0.inches, (123.97 - 4.0).inches),
        )
        val RED_TOWER_BOUNDARY          : Boundary = Boundary(
            "Red_Tower", true,
            Translation2d((609.66 - 4.0).inches, (146.72 - 4.0).inches), Translation2d((609.66 - 4.0).inches, (193.72 + 4.0).inches),
            Translation2d(651.22.inches, (146.72 - 4.0).inches), Translation2d(651.22.inches, (193.72 + 4.0).inches),
        )
    }

    object TargetTranslations {
        val BLUE_HUB                    : Translation2d = Translation2d(4.625.meters, 4.030.meters)
        val RED_HUB                     : Translation2d = Translation2d(11.92.meters, 4.030.meters)
    }
}