package com.team4099.robot2023.subsystems.arm

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.ArmConstants
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.SimulatedMotor
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.asDrivingOverDriven
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ArmIOSim : ArmIO {
  val armSim =
    SingleJointedArmSim(
      DCMotor.getNEO(2),
      ArmConstants.SPROCKET_RATIO.asDrivingOverDriven,
      ArmConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
      ArmConstants.ARM_LENGTH.inMeters,
      ArmConstants.MIN_ROTATION.inRadians,
      ArmConstants.MAX_ROTATION.inRadians,
      true,
    )

  init {
    MotorChecker.add(
      "Arm",
      "Rotation",
      MotorCollection(
        mutableListOf(
          SimulatedMotor(
            armSim,
            "Arm Motor",
          ),
        ),
        60.amps,
        10.celsius,
        45.amps,
        20.celsius
      )
    )
  }

  private val armController =
    PIDController(ArmConstants.PID.SIM_KP, ArmConstants.PID.SIM_KI, ArmConstants.PID.SIM_KD)

  override fun updateInputs(inputs: ArmIO.ArmIOInputs) {
    armSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.armPosition = armSim.angleRads.radians
    inputs.armVelocity = armSim.velocityRadPerSec.radians.perSecond
    inputs.leaderAppliedVoltage = 0.volts
    inputs.followerAppliedVoltage = 0.volts
    inputs.leaderStatorCurrent = armSim.currentDrawAmps.amps
    inputs.followerStatorCurrent = armSim.currentDrawAmps.amps
    inputs.leaderSupplyCurrent = 0.amps
    inputs.followerSupplyCurrent = 0.amps
    inputs.leaderTemp = 0.celsius
    inputs.followerTemp = 0.celsius

    inputs.isSimulated = true
  }

  /**
   * Sets the arm to a desired angle, uses feedforward to account for external forces in the system
   * The armPIDController uses the previously set PID constants and ff to calculate how to get to
   * the desired position
   *
   * @param armPosition the desired angle to set the aerm to
   * @param feedforward the amount of volts to apply for feedforward
   */
  override fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {
    val ff = clamp(feedforward, -12.0.volts, 12.0.volts)
    val feedback = armController.calculate(armSim.angleRads.radians, armPosition)
    armSim.setInputVoltage((ff + feedback).inVolts)
  }

  /**
   * Sets the arm motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the arm motor to
   */
  override fun setArmVoltage(voltage: ElectricalPotential) {
    armSim.setInputVoltage(
      clamp(voltage, -ArmConstants.VOLTAGE_COMPENSATION, ArmConstants.VOLTAGE_COMPENSATION)
        .inVolts
    )
  }

  /**
   * Updates the PID constants using the implementation controller, uses arm sensor to convert from
   * PID constants to motor controller units
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
    armController.setPID(kP, kI, kD)
  }

  /** recalculates the current position of the neo encoder using value from the absolute encoder */
  override fun zeroEncoder(): Boolean {
    return true
  }
}
