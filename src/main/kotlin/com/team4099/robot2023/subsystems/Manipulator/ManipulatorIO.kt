package com.team4099.robot2023.subsystems.Manipulator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface ManipulatorIO {
  class ManipulatorIOInputs : LoggableInputs {
    val statorCurrent = 0.amps
    var isSimulating = false
    var wristPosition: Angle = 0.0.degrees
    var wristVelocity = 0.0.degrees.perSecond

    var wristAppliedVoltage = 0.0.volts
    var wristSupplyCurrent = 0.0.amps
    var wristStatorCurrent = 0.0.amps
    var wristTemp = 0.0.celsius

    var rollerVelocity = 0.0.rotations.perMinute

    var rollerAppliedVoltage = 0.0.volts
    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps
    var rollerTemp = 0.0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("wristPositionDegrees", wristPosition.inDegrees)
      table?.put("wristVelocityDegreesPerSec", wristVelocity.inDegreesPerSecond)
      table?.put("wristAppliedVoltage", wristAppliedVoltage.inVolts)
      table?.put("wristSupplyCurrentAmps", wristSupplyCurrent.inAmperes)
      table?.put("wristStatorCurrentAmps", wristStatorCurrent.inAmperes)
      table?.put("wristTempCelsius", wristTemp.inCelsius)
      table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)
      table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)
      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
      table?.put("rollerTempCelsius", rollerTemp.inCelsius)
      table?.put("statorCurrent", statorCurrent.inAmperes)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("wristPositionDegrees", wristPosition.inDegrees)?.let {
        wristPosition = it.degrees
      }
      table?.getDouble("wristVelocityDegreesPerSec", wristVelocity.inDegreesPerSecond)?.let {
        wristVelocity = it.degrees.perSecond
      }
      table?.getDouble("wristAppliedVoltage", wristAppliedVoltage.inVolts)?.let {
        wristAppliedVoltage = it.volts
      }
      table?.getDouble("wristSupplyCurrentAmps", wristSupplyCurrent.inAmperes)?.let {
        wristSupplyCurrent = it.amps
      }
      table?.getDouble("wristStatorCurrentAmps", wristStatorCurrent.inAmperes)?.let {
        wristStatorCurrent = it.amps
      }
      table?.getDouble("wristTempCelsius", wristTemp.inCelsius)?.let { wristTemp = it.celsius }

      table?.getDouble("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)?.let {
        rollerVelocity = it.rotations.perSecond
      }
      table?.getDouble("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }
      table?.getDouble("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table?.getDouble("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.getDouble("rollerTempCelsius", rollerTemp.inCelsius)?.let { rollerTemp = it.celsius }
    }
  }
  fun updateInputs(inputs: ManipulatorIOInputs)

  /**
   * Sets the voltage of the roller motor but also checks to make sure the voltage doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setRollerVoltage(voltage: ElectricalPotential) {}

  /**
   * Sets the position of the wrist motor, specifically the length of the wrist
   *
   * @param wristPosition the position to set the wrist to
   * @param feedforward changes voltages to compensate for external forces
   */
  fun setWristPosition(wristPosition: Angle, feedforward: ElectricalPotential) {}

  /**
   * Sets the wrist motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the wrist motor to
   */
  fun setWristVoltage(voltage: ElectricalPotential) {}

  /**
   * Updates the PID constants using the implementation controller
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  /** Sets the current encoder position to be the zero value */
  fun zeroEncoder() {}

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  fun setRollerBrakeMode(brake: Boolean) {}

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  fun setWristBrakeMode(brake: Boolean)
}
