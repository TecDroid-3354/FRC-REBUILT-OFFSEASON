// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.tecdroid3354.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera names, must match names configured on coprocessor
    public static String leftCameraName = "limelight-left";
    public static String rightCameraName = "limelight-right";
    public static String backCameraName = "limelight-back";

    // Robot to camera transforms
    // For limelight, you need to configure this in the Web UI (used in real robot) and here (used in simulation)

    // Translation Components
    // x: LL Forward
    // y: LL Right
    // z: LL Up
    public static Transform3d robotToLeftCamera = new Transform3d(-0.041275, -0.322263, 0.157163,
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(17.0), Math.toRadians(90.0)));

    public static Transform3d robotToRightCamera = new Transform3d(-0.041275, 0.322263, 0.157163,
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(17.0), -Math.toRadians(90.0)));

    public static Transform3d robotToBackCamera = new Transform3d(-0.3175, 0.0, 0.4572,
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(20.0), Math.toRadians(180.0)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.18;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others -> Baseline == 1.0, Smaller value == More trust)
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0, // Camera 1
            0.7, // Camera 2
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
