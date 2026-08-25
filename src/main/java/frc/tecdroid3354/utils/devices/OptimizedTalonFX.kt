package frc.tecdroid3354.utils.devices

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoTorqueCurrentFOC
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.AngularVelocityUnit
import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.LinearVelocityUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotMode
import frc.tecdroid3354.utils.Sprocket
import frc.tecdroid3354.utils.controlProfiles.AngularMotionTargets
import frc.tecdroid3354.utils.controlProfiles.LinearMotionTargets
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.hertz
import frc.tecdroid3354.utils.mechanical.Reduction
import frc.tecdroid3354.utils.rotations
import frc.tecdroid3354.utils.rotationsPerSecond
import frc.tecdroid3354.utils.rotationsPerSecondCubed
import frc.tecdroid3354.utils.rotationsPerSecondSquared
import frc.tecdroid3354.utils.safety.MeasureLimits
import java.util.Optional

/**
 * Contains all MotionMagic Request Types used in the team for position control.
 */
enum class OpPositionControlRequests {
    POSITION_VOLTAGE,               // Unlicensed
    POSITION_TORQUE,                // Licensed
    POSITION_EXPO_TORQUE,           // Licensed
    POSITION_DYNAMIC_TORQUE,        // Licensed
    POSITION_DYNAMIC_EXPO_TORQUE,   // Licensed
}

/**
 * Contains all the MotionMagic Request Types used in the team for velocity control.
 */
enum class OpVelocityControlRequests {
    VELOCITY_VOLTAGE,   // Unlicensed
    VELOCITY_TORQUE,    // Licensed
}

//
// DECLARES GLOBAL EXCEPTIONS FOR MOTOR CONTROL
//
private val DYNAMIC_TO_NON_DYNAMIC_METHOD = IllegalCallerException("Tried a dynamic request in a non-dynamic method.")
private val NON_DYNAMIC_TO_DYNAMIC_METHOD = IllegalCallerException("Tried a non-dynamic request in a dynamic method.")
private val MISSING_PARAMETERS_FOR_CONTROL_TYPE = IllegalCallerException("Missing parameter for control type.")
private val REQUIRED_PARAMETERS_NOT_MET =
    IllegalCallerException("No full set of parameters were met to any control request. Motor(s) not commanded.")


// TODO(1) = Make this class an interface and implement it in OpTalonFXLinear, OpTalonFXAngular for modularity??
// TODO(s)? -> { UPDATE G507 }

/**
 * A Phoenix6 TalonFX utility for easier control across subsystems.
 * The intention is that, inside your subsystems, you only need to hand the motors all necessary information instead
 * of worrying about conversions and validation, which is done here.
 * This class also optimizes canbus utilization by only keeping relevant signals active. Motor configuration
 * and follower requests methods are also included.
 */
class OpTalonFX(private val id: Int, private val canBusName: String = "rio"): MotorIO {
    private val motor = KrakenMotors.createDefaultTalon(id ,canBusName)
    private var isFollower: Boolean = false

    private val voltageOutRequest = VoltageOut(0.0) // For SysId Only. Do NOT use with Torque requests.

    //
    // POSITION BASED REQUESTS
    //
    private val motionMagicPositionVoltageRequest = MotionMagicVoltage(0.0) // For Unlicensed use only

    private val motionMagicPositionTorqueRequest = MotionMagicTorqueCurrentFOC(0.0) // Requires license
    private val motionMagicPositionExpoTorqueRequest = MotionMagicExpoTorqueCurrentFOC(0.0) // Requires License
    private val dynamicMotionMagicPositionTorqueRequest = DynamicMotionMagicTorqueCurrentFOC(0.0,0.0,0.0)
    private val dynamicMotionMagicPositionExpoTorqueRequest = DynamicMotionMagicExpoTorqueCurrentFOC(0.0, 0.0, 0.0)

    //
    // VELOCITY BASED REQUESTS
    //
    private val motionMagicVelocityVoltageRequest = MotionMagicVelocityVoltage(0.0) // For Unlicensed use only
    private val motionMagicVelocityTorqueRequest = MotionMagicVelocityTorqueCurrentFOC(0.0) // Requires License

    // Class-level exception to include the ID of the motor.
    private val COMMAND_TO_FOLLOWER_EXCEPTION = IllegalCallerException("Tried to command a follower TalonFX [$id]. Use the lead TalonFX.")

    init {
        optimizeMotorCan() // This method should not be called anywhere else.
    }

    override fun updateInputs(inputs: MotorIO.MotorIOInputs) {
        inputs.isConnected = getIsConnected()

        inputs.closedLoopReference.mut_replace(getClosedLoopTargetPosition())
        inputs.closedLoopReferenceSlope.mut_replace(getClosedLoopTargetVelocity())

        inputs.currentPosition.mut_replace(getPosition())
        inputs.currentVelocity.mut_replace(getVelocity())
        inputs.currentAcceleration.mut_replace(getAcceleration())

        inputs.currentTemperature.mut_replace(getTemperature())
        inputs.outputVoltage.mut_replace(getOutputVoltage())
        inputs.supplyCurrent.mut_replace(getSupplyCurrent())
        inputs.torqueCurrent.mut_replace(getTorqueCurrent())
    }

    //
    // Getters for relevant signals
    //
    fun getPosition(): Angle { return motor.position.value }
    fun getVelocity(): AngularVelocity { return motor.velocity.value }
    fun getAcceleration(): AngularAcceleration { return motor.acceleration.value }
    fun getTemperature(): Temperature { return motor.deviceTemp.value }
    fun getOutputVoltage(): Voltage { return motor.motorVoltage.value }
    fun getSupplyCurrent(): Current { return motor.supplyCurrent.value }
    fun getTorqueCurrent(): Current { return motor.torqueCurrent.value }
    fun getPower(): Double { return motor.get() }
    fun getIsConnected(): Boolean { return if(RobotConstants.ROBOT_MODE == RobotMode.SIM) true else motor.isConnected }

    //
    // Closed Loop References (motor internally-calculated target position and velocity)
    //
    fun getClosedLoopTargetPosition(): Angle { return motor.closedLoopReference.value.rotations }
    fun getClosedLoopTargetVelocity(): AngularVelocity { return motor.closedLoopReferenceSlope.value.rotationsPerSecond }

    /**
     * Non-MotionMagic request. Uses [VoltageOut]; intended only for Sys-Id usage.
     * Warning -> SysId will not give appropriate gains for torque-based subsystems.
     */
    fun sysIdVoltageRequest(voltage: Voltage) {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.setControl(voltageOutRequest
            .withOutput(voltage)
        )
    }

    private fun positionVoltageRequest(position: Angle, slot: Int) {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.setControl(motionMagicPositionVoltageRequest
            .withPosition(position)
            .withSlot(slot)
        )
    }

    private fun positionTorqueRequest(position: Angle, slot: Int) {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.setControl(motionMagicPositionTorqueRequest
            .withPosition(position)
            .withSlot(slot)
        )
    }

    private fun positionExpoTorqueRequest(position: Angle, slot: Int) {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.setControl(motionMagicPositionExpoTorqueRequest
            .withPosition(position)
            .withSlot(slot)
        )
    }

    private fun positionDynamicTorque(position: Angle, velocity: AngularVelocity, acceleration: AngularAcceleration,
                                      jerk: Velocity<AngularAccelerationUnit>,  slot: Int) {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.setControl(dynamicMotionMagicPositionTorqueRequest
            .withPosition(position)
            .withVelocity(velocity)
            .withAcceleration(acceleration)
            .withJerk(jerk)
            .withSlot(slot)
        )
    }

    private fun positionDynamicExpoTorque(position: Angle, velocity: AngularVelocity, expoKV: Double,
                                          expoKA: Double, slot: Int) {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.setControl(dynamicMotionMagicPositionExpoTorqueRequest
            .withPosition(position)
            .withVelocity(velocity)
            .withKV(expoKV)
            .withKA(expoKA)
            .withSlot(slot)
        )
    }

    private fun velocityVoltageRequest(velocity: AngularVelocity, slot: Int) {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.setControl(motionMagicVelocityVoltageRequest
            .withVelocity(velocity)
            .withSlot(slot))
    }

    /**
     * Only difference with voltage is the internal control, which is - surprise - torque based.
     * You will notice this difference when trying to tune the PIDF, SVAG gains for voltage based vs torque
     * based subsystems. This one also requires Phoenix Pro license, make sure your motor has it, or it will throw
     * a sticky fault.
     */
    private fun velocityTorqueRequest(velocity: AngularVelocity, slot: Int) {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.setControl(motionMagicVelocityTorqueRequest
            .withVelocity(velocity)
            .withSlot(slot)
        )
    }

    /**
     * Handles all [OpPositionControlRequests] by calling the corresponding method.
     * In the case that an argument was left as [Optional.empty] and the selected Control Type requires it,
     * an [Exception] will be triggered and the program will stop.
     *
     * This expects to be called from [angularSubsystemPositionRequest], [angularSubsystemPositionDynamicRequest], and
     * [angularSubsystemVelocityRequest] only. See their documentation for instructions on how to use.
     * Linear subsystems convert all linear measures to angular and call the aforementioned methods.
     */
    private fun handlePositionRequest(requestType: OpPositionControlRequests,
                              position: Angle, slot: Int,
                              velocity: Optional<AngularVelocity>, acceleration: Optional<AngularAcceleration>,
                              jerk: Optional<Velocity<AngularAccelerationUnit>>,
                              expoKV: Optional<Double>, expoKA: Optional<Double>
                              ) {
        try {
            when (requestType) {
                OpPositionControlRequests.POSITION_VOLTAGE              -> { positionVoltageRequest(position, slot) }

                OpPositionControlRequests.POSITION_TORQUE               -> { positionTorqueRequest(position, slot) }

                OpPositionControlRequests.POSITION_EXPO_TORQUE          -> { positionExpoTorqueRequest(position, slot) }

                OpPositionControlRequests.POSITION_DYNAMIC_TORQUE       -> {
                    positionDynamicTorque(
                        position, velocity.get(), acceleration.get(), jerk.get(), slot,
                    )
                }

                OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE  -> {
                    positionDynamicExpoTorque(
                        position, velocity.get(), expoKV.get(), expoKA.get(), slot
                    )
                }
            }
        } catch (ex: Exception) {
            throw MISSING_PARAMETERS_FOR_CONTROL_TYPE.also { ex.printStackTrace() }
        }
    }

    /**
     * Calls the corresponding method based on [requestType].
     */
    private fun handleVelocityRequest(requestType: OpVelocityControlRequests, velocity: AngularVelocity, slot: Int) {
        when (requestType) {
            OpVelocityControlRequests.VELOCITY_VOLTAGE  -> { velocityVoltageRequest(velocity, slot) }

            OpVelocityControlRequests.VELOCITY_TORQUE   -> { velocityTorqueRequest(velocity, slot) }
        }
    }

    /**
     * Intended for non-dynamic control types only. Handles limits and subsystem -> motor conversion before calling
     * [handlePositionRequest].
     *
     * The program will throw an [Exception] and stop if you try requesting a dynamic control type through this method,
     * since it does not have the necessary arguments to command them. This is for modularity purposes.
     */
    fun angularSubsystemPositionRequest(controlType: OpPositionControlRequests, subsystemPosition: Angle,
                                        limits: MeasureLimits<AngleUnit>, reduction: Reduction, slot: Int = 0) {
        if (controlType !in listOf( // Make sure user didn't try to use a dynamic request type (not enough parameters)
                OpPositionControlRequests.POSITION_VOLTAGE,
                OpPositionControlRequests.POSITION_TORQUE,
                OpPositionControlRequests.POSITION_EXPO_TORQUE)
            ) { throw DYNAMIC_TO_NON_DYNAMIC_METHOD }

        val subsystemClampedAngle: Angle = limits.coerceIn(subsystemPosition) as Angle
        val motorTransformedAngle: Angle = reduction.unapply(subsystemClampedAngle)
        handlePositionRequest(
            requestType = controlType,
            position = motorTransformedAngle, slot = slot,
            velocity = Optional.empty(), acceleration = Optional.empty(), jerk = Optional.empty(),
            expoKV = Optional.empty(), expoKA = Optional.empty()
        )
    }

    /**
     * Intended for dynamic control types only. Handles limits and subsystem -> motor conversion before calling
     * [handlePositionRequest].
     *
     * The program will throw an [Exception] and stop if you try requesting a non-dynamic control type through this method,
     * since it might be confusing for new users to try and figure out the extra parameters if not using them.
     * This is for modularity purposes.
     *
     * The program will throw an [Exception] and stop if you do not pass the necessary parameters for your requested
     * control type (i.e., calling [OpPositionControlRequests.POSITION_DYNAMIC_TORQUE] without specifying the subsystem
     * motion targets).
     *
     * @param newSubsystemMotionTargets is Required for [OpPositionControlRequests.POSITION_DYNAMIC_TORQUE] only
     * @param expoKV is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     * @param expoKA is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     */
    fun angularSubsystemPositionDynamicRequest(controlType: OpPositionControlRequests, subsystemPosition: Angle,
                                               limits: MeasureLimits<AngleUnit>, reduction: Reduction,
                                               newSubsystemMotionTargets: Optional<AngularMotionTargets>,
                                               expoKV: Optional<Double>, expoKA: Optional<Double>, slot: Int = 0) {
        if (controlType !in listOf( // Make sure user didn't try to use a non-dynamic request type
                OpPositionControlRequests.POSITION_DYNAMIC_TORQUE,
                OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE)
        ) { throw NON_DYNAMIC_TO_DYNAMIC_METHOD }

        val subsystemClampedAngle: Angle = limits.coerceIn(subsystemPosition) as Angle
        val motorTransformedAngle: Angle = reduction.unapply(subsystemClampedAngle)

        if (newSubsystemMotionTargets.isPresent) { // DynamicTorque
            val motorTargets: MotionMagicConfigs = KrakenMotors.configureAngularMotionMagic(
                newSubsystemMotionTargets.get(), reduction
            )
            handlePositionRequest(
                requestType = controlType,
                position = motorTransformedAngle, slot = slot,
                velocity = Optional.of(motorTargets.MotionMagicCruiseVelocity.rotationsPerSecond),
                acceleration = Optional.of(motorTargets.MotionMagicAcceleration.rotationsPerSecondSquared),
                jerk = Optional.of(motorTargets.MotionMagicJerk.rotationsPerSecondCubed),
                expoKV = Optional.empty(), expoKA = Optional.empty()
            )
        }
        else if (expoKV.isPresent and expoKA.isPresent) { // DynamicExpoTorque
            handlePositionRequest(
                requestType = controlType,
                position = motorTransformedAngle, slot = slot,
                velocity = Optional.empty(), acceleration = Optional.empty(), jerk = Optional.empty(),
                expoKV = expoKV, expoKA = expoKA
            )
        } else { throw REQUIRED_PARAMETERS_NOT_MET } // Interrupt program to inform devs of parameter emptiness.
    }

    /**
     * This method applies the limits, converts the target from subsystem units to motor units and calls a private
     * method -> [handleVelocityRequest] for the corresponding control request.
     */
    fun angularSubsystemVelocityRequest(controlType: OpVelocityControlRequests, subsystemVelocity: AngularVelocity,
                                        limits: MeasureLimits<AngularVelocityUnit>, reduction: Reduction, slot: Int = 0) {
        val subsystemClampedVelocity: AngularVelocity = limits.coerceIn(subsystemVelocity) as AngularVelocity
        val motorTransformedVelocity: AngularVelocity = reduction.unapply(subsystemClampedVelocity)

        handleVelocityRequest(controlType, motorTransformedVelocity, slot)
    }

    /**
     * Converts all linear measurements to angular and calls [angularSubsystemPositionRequest].
     *
     * Intended for non-dynamic requests only. An [Exception] will be thrown otherwise.
     */
    fun linearSubsystemPositionRequest(controlType: OpPositionControlRequests, subsystemDisplacement: Distance,
                                       limits: MeasureLimits<DistanceUnit>, sprocket: Sprocket, reduction: Reduction,
                                       slot: Int = 0) {
        val linearToAngularDisplacement =  sprocket.linearDisplacementToAngularDisplacement(subsystemDisplacement)
        val linearToAngularLimits = MeasureLimits(
            sprocket.linearDisplacementToAngularDisplacement(limits.minimum as Distance) ..
                    sprocket.linearDisplacementToAngularDisplacement(limits.maximum as Distance))

        angularSubsystemPositionRequest(controlType, linearToAngularDisplacement, linearToAngularLimits, reduction, slot)
    }

    /**
     * Converts all linear measurements to angular and calls [angularSubsystemPositionDynamicRequest].
     *
     * Intended for dynamic requests only. An [Exception] will be thrown otherwise.
     *
     * @param newSubsystemMotionTargets is Required for [OpPositionControlRequests.POSITION_DYNAMIC_TORQUE] only
     * @param expoKV is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     * @param expoKA is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     */
    fun linearSubsystemPositionDynamicRequest(controlType: OpPositionControlRequests, subsystemDisplacement: Distance,
                                              limits: MeasureLimits<DistanceUnit>, sprocket: Sprocket, reduction: Reduction,
                                              newSubsystemMotionTargets: Optional<LinearMotionTargets>,
                                              expoKV: Optional<Double>, expoKA: Optional<Double>,
                                              slot: Int = 0) {
        val linearToAngularSubsystemDisplacement = sprocket.linearDisplacementToAngularDisplacement(subsystemDisplacement)
        val linearToAngularLimits = MeasureLimits(
            sprocket.linearDisplacementToAngularDisplacement(limits.minimum as Distance) ..
                    sprocket.linearDisplacementToAngularDisplacement(limits.maximum as Distance))

        if (newSubsystemMotionTargets.isPresent) {
            // Just makes it angular, but still in mechanism units
            val linearToAngularMotionTargets = newSubsystemMotionTargets.get().angularMotionTargets(sprocket)

            angularSubsystemPositionDynamicRequest(
                controlType, linearToAngularSubsystemDisplacement, linearToAngularLimits, reduction,
                Optional.of(linearToAngularMotionTargets),
                Optional.empty(), Optional.empty(),
                slot
            )
        }
        else if (expoKV.isPresent and expoKA.isPresent) { // DynamicExpoTorque
            angularSubsystemPositionDynamicRequest(
                controlType, linearToAngularSubsystemDisplacement, linearToAngularLimits, reduction,
                Optional.empty(),
                expoKV, expoKA,
                slot
            )
        } else { throw REQUIRED_PARAMETERS_NOT_MET } // Interrupt program to inform devs of parameter emptiness.
    }

    /**
     * Converts all linear measurements to angular and calls [angularSubsystemVelocityRequest].
     */
    fun linearSubsystemVelocityRequest(controlType: OpVelocityControlRequests, subsystemVelocity: LinearVelocity,
                                       limits: MeasureLimits<LinearVelocityUnit>, sprocket: Sprocket, reduction: Reduction,
                                       slot: Int = 0) {
        val linearToAngularSubsystemVelocity = sprocket.linearVelocityToAngularVelocity(subsystemVelocity)
        val linearToAngularLimits = MeasureLimits(
            sprocket.linearVelocityToAngularVelocity(limits.minimum as LinearVelocity) ..
                    sprocket.linearVelocityToAngularVelocity(limits.maximum as LinearVelocity))

        angularSubsystemVelocityRequest(controlType, linearToAngularSubsystemVelocity, linearToAngularLimits, reduction, slot)
    }

    fun stopMotor() {
        if (isFollower) { throw COMMAND_TO_FOLLOWER_EXCEPTION }

        motor.stopMotor()
    }

    /**
     * Sends a follower request of this motor to the [leadingMotorController].
     * Keep in mind that for subsystem control, you will need to command the [leadingMotorController] and not the one
     * you are calling this method from; doing so will result in an [IllegalCallerException].
     */
    fun follow(leadingMotorController: TalonFX, opposeMaster: MotorAlignmentValue) {
        isFollower = true
        motor.setControl(Follower(leadingMotorController.deviceID, opposeMaster))
    }

    /**
     * First, clears all sticky faults (which happen after a brownout, unlicensed pro feature access, etc.)
     * and then applies the [config]
     */
    fun applyConfigAndClearFaults(config: TalonFXConfiguration) {
        motor.clearStickyFaults()
        motor.configurator.apply(config)
    }

    fun getMotorId(): Int {
        return motor.deviceID
    }

    fun getMotorInstance(): TalonFX {
        return motor
    }

    /**
     * Applies the reduction to get subsystem position
     */
    fun getMotorToAngularSubsystemPosition(reduction: Reduction): Angle {
        return reduction.apply(getPosition())
    }

    /**
     * Applies reduction to get subsystem velocity
     */
    fun getMotorToAngularSubsystemVelocity(reduction: Reduction): AngularVelocity {
        return reduction.apply(getVelocity())
    }

    /**
     * Applies reduction, and then uses the [sprocket] diameter to convert from angular to linear.
     */
    fun getMotorToLinearSubsystemDisplacement(reduction: Reduction, sprocket: Sprocket): Distance {
        return sprocket.angularDisplacementToLinearDisplacement(
            reduction.apply(getPosition())
        )
    }

    /**
     * Applies reduction, and then uses the [sprocket] diameter to convert from angular to linear.
     */
    fun getMotorToLinearSubsystemVelocity(reduction: Reduction, sprocket: Sprocket): LinearVelocity {
        return sprocket.angularVelocityToLinearVelocity(
            reduction.apply(getVelocity())
        )
    }

    /**
     * Just unapplies the reduction to the given measurement.
     */
    fun getAngularSubsystemToMotorPosition(measurement: Angle, reduction: Reduction): Angle {
        return reduction.unapply(measurement)
    }
    /**
     * Just unapplies the reduction to the given measurement.
     */
    fun getAngularSubsystemToMotorVelocity(measurement: AngularVelocity, reduction: Reduction): AngularVelocity {
        return reduction.unapply(measurement)
    }

    fun getAngularSubsystemToMotorAcceleration(measurement: AngularAcceleration, reduction: Reduction): AngularAcceleration {
        return reduction.unapply(measurement)
    }

    /**
     * Conversion as follows:
     *
     * Measurement (linear) -> Sprocket -> Measurement (angular) -> Reduction -> Motor
     */
    fun getLinearSubsystemToMotorPosition(measurement: Distance, reduction: Reduction, sprocket: Sprocket): Angle {
        return reduction.unapply(
            sprocket.linearDisplacementToAngularDisplacement(measurement)
        )
    }

    /**
     * Conversion as follows:
     *
     * Measurement (linear) -> Sprocket -> Measurement (angular) -> Reduction -> Motor
     */
    fun getLinearSubsystemToMotorVelocity(measurement: LinearVelocity, reduction: Reduction, sprocket: Sprocket): AngularVelocity {
        return reduction.unapply(
            sprocket.linearVelocityToAngularVelocity(measurement)
        )
    }

    /**
     * Sets all unnecessary signals to 4Hz.
     * Only signals written inside the method are preserved for real-time usage.
     * Consult an Area Lead / Captain if you consider another signal as necessary.
     */
    private fun optimizeMotorCan() {
        motor.optimizeBusUtilization()
        with(motor) {
            position.setUpdateFrequency(100.0.hertz)
            velocity.setUpdateFrequency(100.0.hertz)
            acceleration.setUpdateFrequency(100.0.hertz)
            motorVoltage.setUpdateFrequency(100.0.hertz)    // Required by followers (Phoenix 6 documentation)
            supplyCurrent.setUpdateFrequency(100.0.hertz)
            controlMode.setUpdateFrequency(100.0.hertz)
            dutyCycle.setUpdateFrequency(100.0.hertz)       // Required by followers (Phoenix 6 documentation)
            torqueCurrent.setUpdateFrequency(100.0.hertz)   // Required by followers (Phoenix 6 documentation)
            version.setUpdateFrequency(100.0.hertz)
        }
    }

    fun coast() {
        motor.setNeutralMode(NeutralModeValue.Coast)
    }

    fun brake() {
        motor.setNeutralMode(NeutralModeValue.Brake)
    }
}
