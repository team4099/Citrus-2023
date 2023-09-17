package com.team4099.robot2023.util.CustomFeedForward


import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.Acceleration
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.LinearGravityFeedforward
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond


class PivotElevatorFeedForward(
  private val kS: StaticFeedforward<Volt>,
  private val kG: LinearGravityFeedforward,
  private val kV: VelocityFeedforward<Meter, Volt>,
  private val kA: AccelerationFeedforward<Meter, Volt>,
  private val initialElevatorAngle: Angle
): ElevatorFeedforward(
  kS.inVolts,
  kG.inVolts,
  kV.inVoltsPerMeterPerSecond,
  kA.inVoltsPerMetersPerSecondPerSecond
) {

  var elevatorAngle = initialElevatorAngle
    set(value) {
      field = value
    }

  fun calculate(velocity: LinearVelocity, acceleration: LinearAcceleration): ElectricalPotential {

    return (kS.inVolts * velocity.sign + kG.inVolts * elevatorAngle.sin + kV.inVoltsPerMeterPerSecond * velocity.inMetersPerSecond + kA.inVoltsPerMeterPerSecondPerSecond * acceleration.inMetersPerSecondPerSecond).volts
  }
}
