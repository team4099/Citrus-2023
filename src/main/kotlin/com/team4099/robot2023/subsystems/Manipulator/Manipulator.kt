package com.team4099.robot2023.subsystems.Manipulator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.requests.Request
import com.team4099.robot2023.config.constants.ArmConstants
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2023.subsystems.superstructure.RobotRequest.ArmRobotRequest as ArmRequest

class Manipulator(private val io: ManipulatorIO) : SubsystemBase() {

  val inputs = ManipulatorIO.ManipulatorIOInputs()

  lateinit var wristFeedforward: ArmFeedforward

  private val kP =
    LoggedTunableValue("Manipulator/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  private val kI =
    LoggedTunableValue(
      "Manipulator/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Manipulator/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  private var prevWristSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.wristPosition, inputs.wristVelocity)

  private val intakeAngle =
    LoggedTunableValue(
      "Manipulator/intakeAngle",
      ManipulatorConstants.INTAKE_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val outtakeAngle =
    LoggedTunableValue(
      "Manipulator/outtakeAngle",
      ManipulatorConstants.OUTTAKE_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val stowedUpAngle =
    LoggedTunableValue(
      "Manipulator/stowedUpAngle",
      ManipulatorConstants.STOWED_UP_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val stowedDownAngle =
    LoggedTunableValue(
      "Manipulator/stowedDownAngle",
      ManipulatorConstants.STOWED_DOWN_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val intakeVoltage =
    LoggedTunableValue(
      "Manipulator/intakeVoltage",
      ManipulatorConstants.INTAKE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  private val outtakeVoltage =
    LoggedTunableValue(
      "Manipulator/outtakeVoltage",
      ManipulatorConstants.OUTTAKE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  private val neutralVoltage =
    LoggedTunableValue(
      "Manipulator/neutralVoltage",
      ManipulatorConstants.NEUTRAL_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  var wristPositionTarget = 0.0.degrees

  var wristVoltageTarget: ElectricalPotential = 0.0.volts

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  private var lastWristPositionTarget = 0.0.degrees

  private var lastRollerVoltage = 0.0.volts

  var isHomed = false

  private var currentWristRequest = ManipulatorRequest.UNINITIALIZED

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentWristRequest == ManipulatorRequests.TARGETING_POSITION &&
          wristProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
          (inputs.wristPosition - wristPositionTarget).absoluteValue <= ManipulatorConstants.TOLERANCE
        )

  val forwardLimitReached: Boolean
    get() = inputs.wristPosition >= ArmConstants.MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.wristPosition <= ArmConstants.MIN_ROTATION

  val canContinueSafely: Boolean
    get() =
      currentWristRequest == ManipulatorRequests.TARGETING_POSITION &&
        (
          ((Clock.fpgaTime - timeProfileGeneratedAt) - wristProfile.totalTime() < 1.0.seconds) ||
            wristProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt)
          ) &&
        (inputs.wristPosition - wristPositionTarget).absoluteValue <= 2.5.degrees

  val openLoopForwardLimitReached: Boolean
    get() = inputs.wristPosition >= ManipulatorConstants.ARM_OPEN_LOOP_MAX_ROTATION

  val openLoopReverseLimitReached: Boolean
    get() = inputs.wristPosition <= ManipulatorConstants.ARM_OPEN_LOOP_MIN_ROTATION

  var lastIntakeRunTime = Clock.fpgaTime

  private var wristConstraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      ManipulatorConstants.MAX_ARM_VELOCITY, ManipulatorConstants.MAX_ARM_ACCELERATION
    )

  private var wristProfile =
    TrapezoidProfile(
      wristConstraints,
      TrapezoidProfile.State(wristPositionTarget, 0.0.degrees.perSecond),
      prevWristSetpoint
    )

  private var timeProfileGeneratedAt = Clock.fpgaTime

  var lastRollerRunTime = Clock.fpgaTime
  val hasCube: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.CUBE_CURRENT_THRESHOLD &&
        (
          inputs.rollerAppliedVoltage == ManipulatorConstants.CUBE_IN ||
            inputs.rollerAppliedVoltage ==
            ManipulatorConstants
              .CUBE_IDLE // TODO checking if their equal is gonna be wrong figure out a
          // better way
          ) &&
        (Clock.fpgaTime - lastRollerRunTime) >=
        ManipulatorConstants.MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }
  val hasCone: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.CONE_CURRENT_THRESHOLD &&
        (
          inputs.rollerAppliedVoltage == ManipulatorConstants.CONE_IN ||
            inputs.rollerAppliedVoltage ==
            ManipulatorConstants
              .CONE_IDLE // TODO checking if their equal is gonna be wrong figure out a
          // better way
          ) &&
        (Clock.fpgaTime - lastRollerRunTime) >=
        ManipulatorConstants.MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(ManipulatorConstants.PID.NEO_KP)
      kI.initDefault(ManipulatorConstants.PID.NEO_KI)
      kD.initDefault(ManipulatorConstants.PID.NEO_KD)

      var wristFeedforward =
        ArmFeedforward(
          ManipulatorConstants.PID.ARM_KS,
          ManipulatorConstants.PID.ARM_KG,
          ManipulatorConstants.PID.ARM_KV,
          ManipulatorConstants.PID.ARM_KA
        )
    } else {
      kP.initDefault(ManipulatorConstants.PID.SIM_KP)
      kI.initDefault(ManipulatorConstants.PID.SIM_KI)
      kD.initDefault(ManipulatorConstants.PID.SIM_KD)

      wristFeedforward =
        ArmFeedforward(
          0.0.volts,
          ManipulatorConstants.PID.ARM_KG,
          ManipulatorConstants.PID.ARM_KV,
          ManipulatorConstants.PID.ARM_KA
        )
    }
  }

  fun generateAndExecuteWristProfile(targetPosition: Length) {}

  fun periodic() {
    io.updateInputs(inputs)
    // Update the PID values when tuning
    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }
  }

  /** @param appliedVoltage Represents the applied voltage of the roller motor */
  fun setRollerVoltage(appliedVoltage: ElectricalPotential) {
    io.setRollerVoltage(appliedVoltage)
  }

  /**
   * Sets the break/idle mode of the wrist
   *
   * @param brake The value that break mode for the wrist will be set as
   */
  fun setWristBrakeMode(brake: Boolean) {
    io.setWristBrakeMode(brake)
    Logger.getInstance().recordOutput("Manipulator/wristBrakeModeEnabled", brake)
  }

  fun zeroWrist() {
    io.zeroEncoder()
  }

  fun setWristVoltage(voltage: ElectricalPotential) {
    io.setWristVoltage(voltage)
  }

  fun setHomeVoltage() {
    io.setWristVoltage(ManipulatorConstants.HOMING_APPLIED_VOLTAGE)
  }

  /**
   * Sets the wrist position using the trapezoidal profile state
   *
   * @param setpoint.position Represents the position the wrist should go to
   * @param setpoint.velocity Represents the velocity the wrist should be at
   */
  private fun isOutOfBounds(velocity: AngularVelocity): Boolean {
    return (velocity > 0.0.degrees.perSecond && forwardLimitReached) ||
      (velocity < 0.0.degrees.perSecond && reverseLimitReached)
  }

  private fun setWristPosition(setpoint: TrapezoidProfile.State<Radian>) {
    // Calculating the acceleration of the wrist
    val wristAngularAcceleration =
      (setpoint.velocity - prevWristSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevWristSetpoint = setpoint
    // Set up the feed forward variable
    val feedforward =
      wristFeedforward.calculate(setpoint.position, setpoint.velocity, wristAngularAcceleration)
    // When the forward or reverse limit is reached, set the voltage to 0
    // Else move the wrist to the setpoint position
    if (isOutOfBounds(setpoint.velocity)) {
      io.setWristVoltage(wristFeedforward.calculate(inputs.wristPosition, 0.degrees.perSecond))
    } else {
      io.setWristPosition(setpoint.position, feedforward)
    }
    Logger.getInstance()
      .recordOutput("Wrist/profileIsOutOfBounds", isOutOfBounds(setpoint.velocity))
    Logger.getInstance().recordOutput("Wrist/wristFeedForward", feedforward.inVolts)
    Logger.getInstance().recordOutput("Wrist/wristTargetPosition", setpoint.position.inDegrees)
    Logger.getInstance()
      .recordOutput("Wrist/wristTargetVelocity", setpoint.velocity.inDegreesPerSecond)
    Logger.getInstance().recordOutput("Wrist/isAtTargetedPosition", isAtTargetedPosition)
  }

  private fun generateWristProfile(): TrapezoidProfile<Radian> {
    val preProfileGenerate = Clock.realTimestamp
    wristProfile =
      TrapezoidProfile(
        wristConstraints,
        TrapezoidProfile.State(wristPositionTarget, 0.0.degrees.perSecond),
        TrapezoidProfile.State(inputs.wristPosition, 0.0.degrees.perSecond)
      )
    val postProfileGenerate = Clock.realTimestamp
    timeProfileGeneratedAt = Clock.fpgaTime

    Logger.getInstance().recordOutput(
      "Wrist/ProfileGenerationTime",
      (postProfileGenerate - preProfileGenerate).inSeconds
    )
    return wristProfile
  }

  private fun executeWristProfile(wristProfile: TrapezoidProfile<Radian>) {
    val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt
    val profileOutput = wristProfile.calculate(timeElapsed)
    setWristPosition(profileOutput)

    Logger.getInstance()
      .recordOutput(
        "Wrist/completedMotionProfile", wristProfile.isFinished(timeElapsed)
      )

    Logger.getInstance()
      .recordOutput("Wrist/profilePositionDegrees", profileOutput.position.inDegrees)
    Logger.getInstance()
      .recordOutput(
        "Wrist/profileVelocityDegreesPerSecond",
        profileOutput.velocity.inDegreesPerSecond
      )
  }

  fun wristZeroRequest(): Request {
    return object : Request() {
      var hasZeroed = false

      override fun act() {
        updateCurrentRequest(ManipulatorRequests.ZEROING_ARM)
        hasZeroed = io.zeroWrist()
      }

      override fun isFinished(): Boolean {
        return hasZeroed
      }
    }
  }

  fun wristOpenLoopRequest(wristVoltage: ElectricalPotential): Request {
    return object : Request() {
      override fun act() {
        updateCurrentRequest(ManipulatorRequests.OPEN_LOOP_REQUEST)
        setWristVoltage(wristVoltage)
      }

      override fun isFinished(): Boolean {
        return false
      }
    }
  }

  fun wristClosedLoopRequest(angle: Angle): Request {
    return object : Request() {
      var wristProfile: TrapezoidProfile<Radian> = generateWristProfile()

      init {
        wristPositionTarget = angle
      }

      override fun act() {
        updateCurrentRequest(ManipulatorRequests.TARGETING_POSITION)
        executeWristProfile(wristProfile)
      }

      override fun isFinished(): Boolean {
        return isAtTargetedPosition
      }
    }
  }

  fun updateCurrentRequest(other: ManipulatorRequests) {
    if (currentWristRequest != other) {
      currentWristRequest = other
    }
  }

  companion object {
    enum class WristRequests {
      UNINITIALIZED,
      ZEROING_ARM,
      TARGETING_POSITION,
      OPEN_LOOP_REQUEST;
    }
  }
}
