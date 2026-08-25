@file:Suppress("unused")

package frc.tecdroid3354.utils

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.LinearAccelerationUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.Units.DegreesPerSecondPerSecond
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Milliseconds
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.RotationsPerSecondPerSecond
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.*

//
// Custom Units
//

data class Pixels(val count: Int) {
    companion object {
        fun of(count: Int) = Pixels(count)
    }
}

data class Percentage(val value: Double)
data class Factor(val value: Double)

//
// Extension Members
//

fun Angle.toRotation2d() = Rotation2d(this)

/**
 * The whole purpose of the below extension members (including 'Inverse Extension Members') is to make
 * more readable working with the Units Library. If you encounter a scenario in which you must convert between
 * Double and some Unit that is not present here, you must include it instead of having a bunch of
 * 'Unit.of()' and 'variable.`in`(Unit)' everywhere.
 * NOTE: Unit.zero() is the exception.
 */

val Int.pixels                          : Pixels                ; get() = Pixels.of(this)
val Double.percent                      : Percentage            ; get() = Percentage(this)
val Double.meters                       : Distance              ; get() = Meters.of(this)
val Double.inches                       : Distance              ; get() = Inches.of(this)
val Double.rotations                    : Angle                 ; get() = Rotations.of(this)
val Double.degrees                      : Angle                 ; get() = Degrees.of(this)
val Double.radians                      : Angle                 ; get() = Radians.of(this)
val Double.hertz                        : Frequency             ; get() = Hertz.of(this)
val Double.seconds                      : Time                  ; get() = Seconds.of(this)
val Double.milliseconds                 : Time                  ; get() = Milliseconds.of(this)
val Double.volts                        : Voltage               ; get() = Volts.of(this)
val Double.amps                         : Current               ; get() = Amps.of(this)
val Double.degreesCelsius               : Temperature           ; get() = Celsius.of(this)
val Double.kilograms                    : Mass                  ; get() = Kilograms.of(this)
val Double.pounds                       : Mass                  ; get() = Pounds.of(this)

// MOI (Moment of Inertia)
val Double.kilogramSquareMeters         : MomentOfInertia       ; get() = KilogramSquareMeters.of(this)

// Velocity
val Double.metersPerSecond              : LinearVelocity        ; get() = MetersPerSecond.of(this)
val Double.degreesPerSecond             : AngularVelocity       ; get() = DegreesPerSecond.of(this)
val Double.radiansPerSecond             : AngularVelocity       ; get() = RadiansPerSecond.of(this)
val Double.rotationsPerSecond           : AngularVelocity       ; get() = RotationsPerSecond.of(this)
val Double.rotationsPerMinute           : AngularVelocity       ; get() = RotationsPerSecond.of(this.div(60.0))

// Acceleration
val Double.metersPerSecondSquared       : LinearAcceleration    ; get() = MetersPerSecondPerSecond.of(this)
val Double.degreesPerSecondSquared      : AngularAcceleration   ; get() = DegreesPerSecondPerSecond.of(this)
val Double.radiansPerSecondSquared      : AngularAcceleration   ; get() = RadiansPerSecondPerSecond.of(this)
val Double.rotationsPerSecondSquared    : AngularAcceleration   ; get() = RotationsPerSecondPerSecond.of(this)

// Jerk
val Double.metersPerSecondCubed     : Velocity<LinearAccelerationUnit>      ; get() = MetersPerSecondPerSecond.per(Second).of(this)
val Double.degreesPerSecondCubed    : Velocity<AngularAccelerationUnit>     ; get() = DegreesPerSecondPerSecond.per(Second).of(this)
val Double.rotationsPerSecondCubed  : Velocity<AngularAccelerationUnit>     ; get() = RotationsPerSecondPerSecond.per(Second).of(this)

//
// Inverse Extension Members
//

val Distance.meters                     : Double                ; get() = this.`in`(Meters)
val Distance.inches                     : Double                ; get() = this.`in`(Inches)
val Angle.rotations                     : Double                ; get() = this.`in`(Rotations)
val Angle.degrees                       : Double                ; get() = this.`in`(Degrees)
val Angle.radians                       : Double                ; get() = this.`in`(Radians)
val Frequency.hertz                     : Double                ; get() = this.`in`(Hertz)
val Time.seconds                        : Double                ; get() = this.`in`(Seconds)
val Time.milliseconds                   : Double                ; get() = this.`in`(Milliseconds)
val Voltage.volts                       : Double                ; get() = this.`in`(Volts)
val Current.amps                        : Double                ; get() = this.`in`(Amps)
val Temperature.degreesCelsius          : Double                ; get() = this.`in`(Celsius)
val Mass.kilograms                      : Double                ; get() = this.`in`(Kilograms)
val Mass.pounds                         : Double                ; get() = this.`in`(Pounds)

// MOI (Moment of Inertia)
val MomentOfInertia.kilogramSquareMeters: Double                ; get() = this.`in`(KilogramSquareMeters)

// Velocity
val LinearVelocity.metersPerSecond      : Double                ; get() = this.`in`(MetersPerSecond)
val AngularVelocity.degreesPerSecond    : Double                ; get() = this.`in`(DegreesPerSecond)
val AngularVelocity.radiansPerSecond    : Double                ; get() = this.`in`(RadiansPerSecond)
val AngularVelocity.rotationsPerSecond  : Double                ; get() = this.`in`(RotationsPerSecond)
val AngularVelocity.rotationsPerMinute  : Double                ; get() = this.`in`(RotationsPerSecond).times(60.0)

// Acceleration
val LinearAcceleration.metersPerSecondSquared     : Double      ; get() = this.`in`(MetersPerSecondPerSecond)
val AngularAcceleration.degreesPerSecondSquared   : Double      ; get() = this.`in`(DegreesPerSecondPerSecond)
val AngularAcceleration.radiansPerSecondSquared   : Double      ; get() = this.`in`(RadiansPerSecondPerSecond)
val AngularAcceleration.rotationsPerSecondSquared : Double      ; get() = this.`in`(RotationsPerSecondPerSecond)

// Jerk
val Velocity<LinearAccelerationUnit>.metersPerSecondCubed       : Double    ; get() = this.`in`(MetersPerSecondPerSecond.per(Second))
val Velocity<AngularAccelerationUnit>.degreesPerSecondCubed     : Double    ; get() = this.`in`(DegreesPerSecondPerSecond.per(Second))
val Velocity<AngularAccelerationUnit>.rotationsPerSecondCubed   : Double    ; get() = this.`in`(RotationsPerSecondPerSecond.per(Second))
