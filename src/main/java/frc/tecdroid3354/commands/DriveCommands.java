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

package frc.tecdroid3354.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.tecdroid3354.constants.RobotConstants;
import frc.tecdroid3354.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static final MutAngle lastDriveAngle = Degrees.mutable(0.0);

    private DriveCommands() {}

    // --------------- --------- -------- ------- --------------- //
    // --------------- AUXILIARY JOYSTICK METHODS --------------- //
    // --------------- --------- -------- ------- --------------- //

    /** Returns the magnitude of the joystick, accounting for its direction. Max drive velocity is not applied. */
    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /** Takes the (x,y) axes of a joystick and returns the angle it's pointing at for the robot, accounting for alliance.
     * Make sure to pass the values without any pre-processing, conventions are handled in the method. */
    public static Angle getAngleFromJoystick(double x, double y) {
        if (abs(-y) < 0.3 && abs(x) < 0.3) return lastDriveAngle;

        double atan2Rad = atan2( // Gets the "raw" angle from the joysticks
                RobotConstants.INSTANCE.getIS_RED_ALLIANCE().getAsBoolean()
                        ? -y
                        : y,
                RobotConstants.INSTANCE.getIS_RED_ALLIANCE().getAsBoolean()
                        ? x
                        : -x
        ) + (Math.PI / 2); // Corrects due to joystick's own reference frame

        lastDriveAngle.mut_replace(Radians.of(atan2Rad));

        return lastDriveAngle;
    }

    /** Useful for the drive to remember the last rotation of an autonomous routine */
    public static void overrideLastJoystickAngle(Rotation2d lastJoystickAngle) {
        lastDriveAngle.mut_replace(lastJoystickAngle.getMeasure());
    }

    // --------------- --------- ------- ------- --------------- //
    // --------------- AUXILIARY CHASSIS METHODS --------------- //
    // --------------- --------- ------- ------- --------------- //

    /** Constructs a vector from robot to target and returns its angle plus the headingOffset, if present.
     * @param rememberTarget Set this to true if you'll command the drive with the resultant target, false otherwise */
    public static Rotation2d getAngleFromRobotToTarget(Pose2d robotPose, Translation2d fieldToTarget,
                                                       Optional<Rotation2d> headingOffset, boolean rememberTarget) {
        Translation2d robotToTargetVector = fieldToTarget.minus(robotPose.getTranslation());

        Rotation2d targetAngle = Rotation2d.fromRadians(
          atan2(robotToTargetVector.getY(), robotToTargetVector.getX())
        ).plus(headingOffset.orElse(new Rotation2d()));

        if (rememberTarget) lastDriveAngle.mut_replace(targetAngle.getMeasure());

        return targetAngle;
    }

    /** Creates a vector from robot to target and returns the projection of the velocity vector onto the distance unit vector
     * <p>  - Positive result = Driving towards the target </p>
     * <p>  - Negative result = Driving away from target </p>
     * */
    public static LinearVelocity getRobotRadialVelocity(ChassisSpeeds fieldRelativeSpeeds,
                                                        Pose2d robotPose, Translation2d fieldToTarget) {
        Translation2d robotToTargetVector = fieldToTarget.minus(robotPose.getTranslation());
        double robotToTargetDistance = robotToTargetVector.getNorm();

        // You're over the target, this would cause a division by zero
        if (robotToTargetDistance <= 1e-5) return MetersPerSecond.zero();

        Translation2d radialUnitVector = robotToTargetVector.div(robotToTargetDistance);

        Translation2d velocityVector = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
        double radialVectorNorm = velocityVector.dot(radialUnitVector);

        Translation2d radialVectorEnd = robotPose.getTranslation().plus(radialUnitVector.times(radialVectorNorm));

        // Allows arrow-like visualization in AdvantageScope
        Logger.recordOutput("Vector_Speeds/Radial", new Translation3d(robotPose.getX(), robotPose.getY(), 0.5),
                new Translation3d(radialVectorEnd.getX(), radialVectorEnd.getY(), 0.5));

        return MetersPerSecond.of(radialVectorNorm);
    }

    /** Rotates the robot -> target vector 90deg CCW and returns the projection of the velocity vector onto the tangential unit vector
     * <p>  - Positive result = Orbiting the target Counter-Clockwise </p>
     * <p>  - Negative result = Orbiting the target Clockwise </p>
     * */
    public static LinearVelocity getRobotTangentialVelocity(ChassisSpeeds fieldRelativeSpeeds,
                                                            Pose2d robotPose, Translation2d fieldToTarget) {
        Translation2d robotToTargetVector = fieldToTarget.minus(robotPose.getTranslation());
        double robotToTargetDistance = robotToTargetVector.getNorm();

        // You're over the target, this would cause a division by zero
        if (robotToTargetDistance <= 1e-5) return MetersPerSecond.zero();

        Translation2d radialUnitVector = robotToTargetVector.div(robotToTargetDistance);
        // Rotates the distance unit vector 90deg Counter-Clockwise to get a unit vector perpendicular to the radius.
        Translation2d tangentialUnitVector = radialUnitVector.rotateBy(Rotation2d.kCCW_90deg);

        Translation2d velocityVector = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
        double tangentialVectorNorm = velocityVector.dot(tangentialUnitVector);

        Translation2d tangentialVectorEnd = robotPose.getTranslation().plus(tangentialUnitVector.times(tangentialVectorNorm));

        // Allows arrow-like visualization in AdvantageScope
        Logger.recordOutput("Vector_Speeds/Tangential", new Translation3d(robotPose.getX(), robotPose.getY(), 0.5),
                new Translation3d(tangentialVectorEnd.getX(), tangentialVectorEnd.getY(), 0.5));

        return MetersPerSecond.of(tangentialVectorNorm);
    }

    // --------------- ----- -------- --------------- //
    // --------------- DRIVE COMMANDS --------------- //
    // --------------- ----- -------- --------------- //

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static Command joystickDrive(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    // --------------- ---------------- -------- --------------- //
    // --------------- CHARACTERIZATION COMMANDS --------------- //
    // --------------- ---------------- -------- --------------- //

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta
                        Commands.run(() -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
