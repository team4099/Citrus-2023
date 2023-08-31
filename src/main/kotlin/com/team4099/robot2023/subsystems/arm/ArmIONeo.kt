package com.team4099.robot2023.subsystems.arm

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.ArmConstants
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.Neo
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.asDrivenOverDriving
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.IEEErem
import kotlin.math.absoluteValue

object ArmIONeo : ArmIO {
  private val leaderSparkMax =
    CANSparkMax(Constants.Arm.LEADER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderSensor =
    sparkMaxAngularMechanismSensor(
      leaderSparkMax,
      ArmConstants.SPROCKET_RATIO.asDrivenOverDriving,
      ArmConstants.VOLTAGE_COMPENSATION
    )

  private val followerSparkMax =
    CANSparkMax(Constants.Arm.FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderPIDController: SparkMaxPIDController = leaderSparkMax.pidController

  private val armEncoder = leaderSparkMax.encoder

  private val throughBoreEncoder = DutyCycleEncoder(Constants.Arm.REV_ENCODER_PORT)

  private val armPIDController: SparkMaxPIDController = leaderSparkMax.pidController

  // Uses the absolute encoder position to calculate the arm position
  private val armAbsolutePosition: Angle
    get() {
      return (throughBoreEncoder.absolutePosition.rotations - ArmConstants.ABSOLUTE_ENCODER_OFFSET)
        .inDegrees.IEEErem(360.0)
        .degrees
    }

  init {
    // Resetting motor
    leaderSparkMax.restoreFactoryDefaults()
    followerSparkMax.restoreFactoryDefaults()

    leaderSparkMax.clearFaults()
    followerSparkMax.clearFaults()

    // Basic settings
    leaderSparkMax.enableVoltageCompensation(ArmConstants.VOLTAGE_COMPENSATION.inVolts)
    followerSparkMax.enableVoltageCompensation(ArmConstants.VOLTAGE_COMPENSATION.inVolts)

    followerSparkMax.inverted = Constants.Arm.INVERT_FOLLOWER

    leaderSparkMax.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT.inAmperes.toInt())
    followerSparkMax.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT.inAmperes.toInt())

    leaderSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    followerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake

    // Makes follower motor output exact same power as leader
    followerSparkMax.follow(leaderSparkMax)

    leaderPIDController.ff = 0.0

    leaderSparkMax.burnFlash()
    followerSparkMax.burnFlash()

    MotorChecker.add(
      "Arm",
      "Pivot",
      MotorCollection(
        mutableListOf(
          Neo(leaderSparkMax, "Pivot Leader Motor"),
          Neo(followerSparkMax, "Pivot Follower Motor")
        ),
        ArmConstants.ARM_CURRENT_LIMIT,
        30.celsius,
        ArmConstants.ARM_CURRENT_LIMIT - 0.amps,
        90.celsius
      ),
    )
  }

  override fun updateInputs(inputs: ArmIO.ArmIOInputs) {
    inputs.armPosition = leaderSensor.position
    inputs.armAbsoluteEncoderPosition = armAbsolutePosition
    inputs.armVelocity = leaderSensor.velocity

    // voltage in * percent out
    inputs.leaderAppliedVoltage = leaderSparkMax.busVoltage.volts * leaderSparkMax.appliedOutput

    inputs.leaderStatorCurrent = leaderSparkMax.outputCurrent.amps

    // BatteryVoltage * SupplyCurrent = percentOutput * BatteryVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent

    inputs.leaderSupplyCurrent =
      inputs.leaderStatorCurrent * leaderSparkMax.appliedOutput.absoluteValue

    inputs.leaderTemp = leaderSparkMax.motorTemperature.celsius

    // voltage in * percent out
    inputs.followerAppliedVoltage = leaderSparkMax.busVoltage.volts * followerSparkMax.appliedOutput

    inputs.followerStatorCurrent = followerSparkMax.outputCurrent.amps

    inputs.followerSupplyCurrent =
      inputs.followerStatorCurrent * followerSparkMax.appliedOutput.absoluteValue

    inputs.followerTemp = followerSparkMax.motorTemperature.celsius

    inputs.leaderRawPosition = leaderSparkMax.encoder.position.degrees

    inputs.followerRawPosition = followerSparkMax.encoder.position.degrees
  }

  /*
  Zeroes the encoder of the leader spark max (the arm encoder) to make sure it's in line
  with the through bore encoder.
   */
  override fun zeroEncoder() {
    armEncoder.position = leaderSensor.positionToRawUnits(armAbsolutePosition)
  }

  /**
   * Sets the arm motor brake mode
   *
   * @param brake If it should brake
   */
  override fun setArmBrakeMode(brake: Boolean) {
    if (brake) {
      leaderSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      leaderSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }
}
