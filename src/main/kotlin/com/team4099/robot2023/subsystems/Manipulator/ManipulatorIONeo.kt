package com.team4099.robot2023.subsystems.Manipulator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inPercentOutputPerSecond
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.asDrivenOverDriving
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor

object ManipulatorIONeo : ManipulatorIO {
  private val rollerSparkMax =
    CANSparkMax(Constants.Manipulator.ROLLER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val wristSparkMax =
    CANSparkMax(Constants.Manipulator.WRIST_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val rollerSensor =
    sparkMaxAngularMechanismSensor(
      rollerSparkMax,
      ManipulatorConstants.ROLLER_GEAR_RATIO.asDrivenOverDriving,
      ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION,
    )

  private val wristSensor =
    sparkMaxAngularMechanismSensor(
      wristSparkMax,
      ManipulatorConstants.WRIST_GEAR_RATIO.asDrivenOverDriving,
      ManipulatorConstants.WRIST_VOLTAGE_COMPENSATION,
    )

  private val wristPIDController: SparkMaxPIDController = wristSparkMax.pidController

  init {
    rollerSparkMax.restoreFactoryDefaults()
    rollerSparkMax.clearFaults()

    rollerSparkMax.enableVoltageCompensation(ManipulatorConstants.VOLTAGE_COMPENSATION.inVolts)
    rollerSparkMax.setSmartCurrentLimit(ManipulatorConstants.ROLLER_CURRENT_LIMIT.inAmperes.toInt())
    rollerSparkMax.inverted = ManipulatorConstants.ROLLER_MOTOR_INVERTED

    rollerSparkMax.openLoopRampRate = ManipulatorConstants.ROLLER_RAMP_RATE.inPercentOutputPerSecond
    rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast

    rollerSparkMax.burnFlash()

    wristSparkMax.restoreFactoryDefaults()
    wristSparkMax.clearFaults()

    wristSparkMax.enableVoltageCompensation(ManipulatorConstants.WRIST_VOLTAGE_COMPENSATION.inVolts)
    wristSparkMax.setSmartCurrentLimit(ManipulatorConstants.WRIST_CURRENT_LIMIT.inAmperes.toInt())
    wristSparkMax.inverted = ManipulatorConstants.WRIST_MOTOR_INVERTED
    wristSparkMax.idleMode = CANSparkMax.IdleMode.kBrake

    wristSparkMax.burnFlash()
  }

  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = rollerSparkMax.busVoltage.volts * rollerSparkMax.appliedOutput
    inputs.rollerStatorCurrent = rollerSparkMax.outputCurrent.amps

    // BusVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BusVoltage
    // SupplyCurrent = (percentOutput * BusVoltage / BusVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.rollerSupplyCurrent = inputs.rollerStatorCurrent * rollerSparkMax.appliedOutput
    inputs.rollerTemp = rollerSparkMax.motorTemperature.celsius

    inputs.wristPosition = wristSensor.position
    inputs.wristVelocity = wristSensor.velocity
    inputs.wristAppliedVoltage = wristSparkMax.busVoltage.volts * wristSparkMax.appliedOutput
    inputs.wristStatorCurrent = wristSparkMax.outputCurrent.amps

    // same math as  rollersupplycurrent
    inputs.wristSupplyCurrent = inputs.wristStatorCurrent * wristSparkMax.appliedOutput
  }

  /**
   * Sets the roller motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setRollerVoltage(voltage: ElectricalPotential) {
    rollerSparkMax.setVoltage(
      clamp(
        voltage,
        -ManipulatorConstants.VOLTAGE_COMPENSATION,
        ManipulatorConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  /**
   * Sets the wrist motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the wrist motor to
   */
  override fun setWristVoltage(voltage: ElectricalPotential) {
    wristSparkMax.setVoltage(
      clamp(
        voltage,
        -ManipulatorConstants.VOLTAGE_COMPENSATION,
        ManipulatorConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  /**
   * Sets the wrist to a desired angle, uses feedforward to account for external forces in the
   * system The wristPIDController uses the previously set PID constants and ff to calculate how to
   * get to the desired position
   *
   * @param wristPosition the desired angle to set the aerm to
   * @param feedforward the amount of volts to apply for feedforward
   */
  override fun setWristPosition(wristPosition: Angle, feedforward: ElectricalPotential) {
    wristPIDController.ff = feedforward.inVolts
    wristPIDController.setReference(
      wristSensor.positionToRawUnits(wristPosition), CANSparkMax.ControlType.kPosition
    )
  }

  /**
   * Updates the PID constants using the implementation controller, uses wrist sensor to convert
   * from PID constants to motor controller units
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    wristPIDController.p = wristSensor.proportionalPositionGainToRawUnits(kP)
    wristPIDController.i = wristSensor.integralPositionGainToRawUnits(kI)
    wristPIDController.d = wristSensor.derivativePositionGainToRawUnits(kD)
  }

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }

  /**
   * Sets the wrist motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setWristBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }
}
