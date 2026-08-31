package frc.tecdroid3354.constants

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
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

        val UPPER_BLUE_ASSIST           : Translation2d = Translation2d(3.625.meters, 5.42.meters)
        val BOTTOM_BLUE_ASSIST          : Translation2d = Translation2d(3.625.meters, 2.64.meters)

        val UPPER_RED_ASSIST            : Translation2d = Translation2d(12.92.meters, 5.42.meters)
        val BOTTOM_RED_ASSIST           : Translation2d = Translation2d(12.92.meters, 2.64.meters)
    }

    object FieldDimensions {
        val X_AXIS_WIDTH                    : Distance = 16.51.meters
        val Y_AXIS_HEIGHT                   : Distance = 8.04.meters

        val NEUTRAL_ZONE_BLUE_END_X_AXIS    : Distance = 5.19.meters
        val NEUTRAL_ZONE_RED_END_X_AXIS     : Distance = 11.35.meters
    }
}