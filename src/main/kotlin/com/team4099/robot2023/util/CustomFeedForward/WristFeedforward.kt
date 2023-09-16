package com.team4099.robot2023.util.CustomFeedForward

import edu.wpi.first.math.controller.ArmFeedforward
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint
import org.team4099.lib.units.Acceleration
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.LinearGravityFeedforward
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecondPerSecond
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRadiansPerSecondPerSecond


class WristFeedforward
  (
  private val kS: StaticFeedforward<Volt>,
  private val kG: LinearGravityFeedforward,
  private val kV: VelocityFeedforward<Radian, Volt>,
  private val kA: AccelerationFeedforward<Radian, Volt>,
  private val initialElevatorAngle: Angle
): ArmFeedforward(
kS.inVolts,
kG.inVolts,
kV.inVoltsPerRadianPerSecond,
kA.inVoltsPerRadianPerSecondPerSecond
) {

  var elevatorAngle = initialElevatorAngle
    set(value) {
      field = value
    }

  fun calculate(position: Angle, velocity: AngularVelocity, acceleration: AngularAcceleration): ElectricalPotential {
    return (kS.inVolts * velocity.sign + kG.inVolts * (position.sin * elevatorAngle.sin + position.cos * initialElevatorAngle.cos) + kV.inVoltsPerRadianPerSecond * velocity.inRadiansPerSecond + kA.inVoltsPerRadianPerSecondPerSecond * acceleration.inRadiansPerSecondPerSecond).volts
  }
}
