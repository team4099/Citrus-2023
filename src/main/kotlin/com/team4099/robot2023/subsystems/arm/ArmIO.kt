package com.team4099.robot2023.subsystems.arm

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
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

interface ArmIO {
  class ArmIOInputs : LoggableInputs {
    var armPosition: Angle = 0.0.degrees
    var armVelocity = 0.0.degrees.perSecond

    var armAbsoluteEncoderPosition = 0.0.degrees

    var leaderAppliedVoltage = 0.0.volts
    var leaderSupplyCurrent = 0.0.amps
    var leaderStatorCurrent = 0.0.amps
    var leaderTemp = 0.0.celsius
    var leaderRawPosition = 0.0.degrees

    var followerAppliedVoltage = 0.0.volts
    var followerSupplyCurrent = 0.0.amps
    var followerStatorCurrent = 0.0.amps
    var followerTemp = 0.0.celsius
    var followerRawPosition = 0.0.degrees

    var isSimulated = false

    override fun toLog(table: LogTable?) {
      table?.put("armPositionDegrees", armPosition.inDegrees)
      table?.put("armAbsoluteEncoderPositionDegrees", armAbsoluteEncoderPosition.inDegrees)
      table?.put("armVelocityDegreesPerSec", armVelocity.inDegreesPerSecond)

      table?.put("leaderAppliedVoltage", leaderAppliedVoltage.inVolts)
      table?.put("followerAppliedVoltage", followerAppliedVoltage.inVolts)

      table?.put("leaderSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)
      table?.put("followerSupplyCurrentAmps", followerSupplyCurrent.inAmperes)

      table?.put("leaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)
      table?.put("followerStatorCurrentAmps", followerStatorCurrent.inAmperes)

      table?.put("leaderTempCelsius", leaderTemp.inCelsius)
      table?.put("followerTempCelsius", followerTemp.inCelsius)

      table?.put("leaderRawPositionDegrees", leaderRawPosition.inDegrees)
      table?.put("followerRawPositionDegrees", followerRawPosition.inDegrees)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("armPositionDegrees", armPosition.inDegrees)?.let {
        armPosition = it.degrees
      }
      table?.getDouble("armAbsoluteEncoderPositionDegrees", armAbsoluteEncoderPosition.inDegrees)
        ?.let { armAbsoluteEncoderPosition = it.degrees }
      table?.getDouble("armVelocityDegreesPerSec", armVelocity.inDegreesPerSecond)?.let {
        armVelocity = it.degrees.perSecond
      }
      table?.getDouble("leaderAppliedVoltage", leaderAppliedVoltage.inVolts)
        ?.let { leaderAppliedVoltage = it.volts }
      table?.getDouble("followerAppliedVoltage", followerAppliedVoltage.inVolts)
        ?.let { followerAppliedVoltage = it.volts }
      table?.getDouble("leaderSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)
        ?.let { leaderSupplyCurrent = it.amps }
      table?.getDouble("followerSupplyCurrentAmps", followerSupplyCurrent.inAmperes)
        ?.let { followerSupplyCurrent = it.amps }
      table?.getDouble("leaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)
        ?.let { leaderStatorCurrent = it.amps }
      table?.getDouble("followerStatorCurrentAmps", followerStatorCurrent.inAmperes)
        ?.let { followerStatorCurrent = it.amps }
      table?.getDouble("leaderTempCelsius", leaderTemp.inCelsius)
        ?.let { leaderTemp = it.celsius }
      table?.getDouble("followerTempCelsius", followerTemp.inCelsius)
        ?.let { followerTemp = it.celsius }
      table?.getDouble("leaderRawPositionDegrees", leaderRawPosition.inDegrees)
        ?.let { leaderRawPosition = it.degrees }
      table?.getDouble("followerRawPositionDegrees", followerRawPosition.inDegrees)
        ?.let { followerRawPosition = it.degrees }
    }
  }

  fun updateInputs(inputs: ArmIOInputs)

  /**
   * Sets the position of the arm motor, specifically the length of the arm
   *
   * @param armPosition the position to set the arm to
   * @param feedforward changes voltages to compensate for external forces
   */
  fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {}

  /**
   * Sets the arm motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the arm motor to
   */
  fun setArmVoltage(voltage: ElectricalPotential) {}

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
   * Sets the arm motor brake mode
   *
   * @param brake if it brakes
   */
  fun setArmBrakeMode(brake: Boolean) {}
}
