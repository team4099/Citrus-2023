package com.team4099.robot2023.subsystems.Manipulator
import com.revrobotics.CANSparkMax
import com.team4099.lib.math.clamp
import com.team4099.lib.sim.vision.MathUtils.clamp
import com.team4099.robot2023.config.constants.Constants.Universal
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
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
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object ManipulatorIOSim : ManipulatorIO{
  private val rollerSim = FlywheelSim(
    DCMotor.getNEO(1, ),
    ManipulatorConstants.ROLLER_GEAR_RATIO.asDrivingOverDriven,
    ManipulatorConstants.ROLLER_MOMENT_OF_INERTIA.inKilogramsMeterSquared
  )
  private val wristSim = SingleJointedArmSim(
    DCMotor.getNEO(1, ),
    ManipulatorConstants.WRIST_GEAR_RATIO.asDrivingOverDriven,
    ManipulatorConstants.WRIST_MOMENT_OF_INERTIA.inKilogramsMeterSquared,
    ManipulatorConstants.WRIST_LENGTH.inMeters,
    ManipulatorConstants.WRIST_MIN_ROTATION.inRadians,
    ManipulatorConstants.WRIST_MAX_ROTATION.inRadians,
    false
  )

  private var rollerVoltage = 0.volts
  private var wristVoltage = 0.volts

  private val wristController =
    PIDController(
      ManipulatorConstants.PID.SIM_KP,
      ManipulatorConstants.PID.SIM_KI,
      ManipulatorConstants.PID.SIM_KD,
    )


  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    rollerSim.update(Universal.LOOP_PERIOD_TIME.inSeconds)
    wristSim.update(Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
    inputs.rollerTemp = 0.celsius
    inputs.rollerStatorCurrent = 0.amps
    inputs.rollerSupplyCurrent = 0.amps
    inputs.rollerAppliedVoltage = rollerVoltage

    inputs.wristPosition = wristSim.angleRads.radians
    inputs.wristVelocity = wristSim.velocityRadPerSec.radians.perSecond
    inputs.wristSupplyCurrent = 0.amps
    inputs.wristStatorCurrent = 0.amps
    inputs.wristAppliedVoltage = wristVoltage
    inputs.wristTemp = 0.celsius

    inputs.isSimulating = true
  }

  override fun setWristVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp (
        voltage,
        -ManipulatorConstants.WRIST_VOLTAGE_COMPENSATION,
        ManipulatorConstants.WRIST_VOLTAGE_COMPENSATION
      )
    wristVoltage = clampedVoltage
    wristSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setRollerVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp (
        voltage,
        -ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION,
        ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION
      )
    rollerVoltage = clampedVoltage
    rollerSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setWristPosition(wristPosition: Angle, feedforward: ElectricalPotential) {
    val feedback = wristController.calculate(wristSim.angleRads.radians, wristPosition)
    setWristVoltage(feedforward + feedback)
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    wristController.setPID(kP, kI, kD)
  }

  override fun zeroEncoder() {}

  override fun setWristBrakeMode(brake: Boolean) {}

  override fun setRollerBrakeMode(brake: Boolean) {}

}
