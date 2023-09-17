package com.team4099.robot2023.subsystems.wrist

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.requests.Request
import com.team4099.robot2023.config.constants.ArmConstants
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.util.CustomFeedForward.WristFeedforward
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Length
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

class Wrist(private val io: WristIO) : SubsystemBase() {

  val inputs = WristIO.WristIOInputs()

  lateinit var wristFeedforward: WristFeedforward

  private val kP =
    LoggedTunableValue("Wrist/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  private val kI =
    LoggedTunableValue(
      "Wrist/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Wrist/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  private var prevWristSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.wristPosition, inputs.wristVelocity)

  private val intakeAngle =
    LoggedTunableValue(
      "Wrist/intakeAngle",
      WristConstants.INTAKE_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val outtakeAngle =
    LoggedTunableValue(
      "Wrist/outtakeAngle",
      WristConstants.OUTTAKE_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val stowedUpAngle =
    LoggedTunableValue(
      "Wrist/stowedUpAngle",
      WristConstants.STOWED_UP_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val stowedDownAngle =
    LoggedTunableValue(
      "Wrist/stowedDownAngle",
      WristConstants.STOWED_DOWN_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val intakeVoltage =
    LoggedTunableValue(
      "Wrist/intakeVoltage",
      WristConstants.INTAKE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  private val outtakeVoltage =
    LoggedTunableValue(
      "Wrist/outtakeVoltage",
      WristConstants.OUTTAKE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  private val neutralVoltage =
    LoggedTunableValue(
      "Wrist/neutralVoltage",
      WristConstants.NEUTRAL_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  var wristPositionTarget = 0.0.degrees

  var wristVoltageTarget: ElectricalPotential = 0.0.volts

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  private var lastWristPositionTarget = 0.0.degrees

  private var lastRollerVoltage = 0.0.volts

  var isHomed = false

  private var currentWristRequest = WristRequests.UNINITIALIZED

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentWristRequest == WristRequests.TARGETING_POSITION &&
          wristProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
          (inputs.wristPosition - wristPositionTarget).absoluteValue <= WristConstants.TOLERANCE
        )

  val forwardLimitReached: Boolean
    get() = inputs.wristPosition >= WristConstants.MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.wristPosition <= WristConstants.MIN_ROTATION

  val canContinueSafely: Boolean
    get() =
      currentWristRequest == WristRequests.TARGETING_POSITION &&
        (
          ((Clock.fpgaTime - timeProfileGeneratedAt) - wristProfile.totalTime() < 1.0.seconds) ||
            wristProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt)
          ) &&
        (inputs.wristPosition - wristPositionTarget).absoluteValue <= 2.5.degrees

  val openLoopForwardLimitReached: Boolean
    get() = inputs.wristPosition >= WristConstants.ARM_OPEN_LOOP_MAX_ROTATION

  val openLoopReverseLimitReached: Boolean
    get() = inputs.wristPosition <= WristConstants.ARM_OPEN_LOOP_MIN_ROTATION

  var lastIntakeRunTime = Clock.fpgaTime

  private var wristConstraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      WristConstants.MAX_ARM_VELOCITY, WristConstants.MAX_ARM_ACCELERATION
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
      return inputs.rollerStatorCurrent >= WristConstants.CUBE_CURRENT_THRESHOLD &&
        (
          inputs.rollerAppliedVoltage == WristConstants.CUBE_IN ||
            inputs.rollerAppliedVoltage ==
            WristConstants
              .CUBE_IDLE // TODO checking if their equal is gonna be wrong figure out a
          // better way
          ) &&
        (Clock.fpgaTime - lastRollerRunTime) >=
        WristConstants.WRIST_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }
  val hasCone: Boolean
    get() {
      return inputs.rollerStatorCurrent >= WristConstants.CONE_CURRENT_THRESHOLD &&
        (
          inputs.rollerAppliedVoltage == WristConstants.CONE_IN ||
            inputs.rollerAppliedVoltage ==
            WristConstants
              .CONE_IDLE // TODO checking if their equal is gonna be wrong figure out a
          // better way
          ) &&
        (Clock.fpgaTime - lastRollerRunTime) >=
        WristConstants.WRIST_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(WristConstants.PID.NEO_KP)
      kI.initDefault(WristConstants.PID.NEO_KI)
      kD.initDefault(WristConstants.PID.NEO_KD)

      var wristFeedforward =
        WristFeedforward(
          WristConstants.PID.ARM_KS,
          WristConstants.PID.ARM_KG,
          WristConstants.PID.ARM_KV,
          WristConstants.PID.ARM_KA,
          ArmConstants.MIN_ROTATION
        )
    } else {
      kP.initDefault(WristConstants.PID.SIM_KP)
      kI.initDefault(WristConstants.PID.SIM_KI)
      kD.initDefault(WristConstants.PID.SIM_KD)

      wristFeedforward =
        WristFeedforward(
          0.0.volts,
          WristConstants.PID.ARM_KG,
          WristConstants.PID.ARM_KV,
          WristConstants.PID.ARM_KA,
          ArmConstants.MIN_ROTATION
        )
    }
  }


  override fun periodic() {
    io.updateInputs(inputs)
    // Update the PID values when tuning
    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }
    Logger.getInstance().processInputs("Wrist", inputs)
    Logger.getInstance().recordOutput("Wrist/currentRequest", currentWristRequest.name)

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
    Logger.getInstance().recordOutput("Wrist/wristBrakeModeEnabled", brake)
  }

  fun zeroWrist() {
    io.zeroEncoder()
  }

  fun setWristVoltage(voltage: ElectricalPotential) {
    if ((openLoopForwardLimitReached && voltage > 0.0.volts) ||
      (openLoopReverseLimitReached && voltage < 0.0.volts)
    ) {
      io.setWristVoltage(0.0.volts)
    } else {
      io.setWristVoltage(voltage)
    }
  }

  fun setHomeVoltage() {
    io.setWristVoltage(WristConstants.HOMING_APPLIED_VOLTAGE)
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
      io.setWristVoltage(wristFeedforward.calculate(inputs.wristPosition, 0.degrees.perSecond, 0.degrees.perSecond.perSecond))
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

  fun isAtHome() : Boolean {
    if (inputs.wristStatorCurrent < WristConstants.HOMING_STALL_CURRENT) {
      lastHomingStatorCurrentTripTime = Clock.fpgaTime
    }
    return (inputs.isSimulating ||
      (
        isHomed &&
          inputs.wristStatorCurrent >= WristConstants.HOMING_STALL_CURRENT &&
          (Clock.fpgaTime - lastHomingStatorCurrentTripTime) >=
          WristConstants.HOMING_STALL_TIME_THRESHOLD
        )
      )
  }

  fun wristHomeRequest(): Request {
    return object : Request() {
      var hasHomed = false

      override fun act() {
        updateCurrentRequest(WristRequests.HOME)
        if (isAtHome()) {
          hasHomed = true
          inputs.wristPosition = 215.degrees
        } else {
          setHomeVoltage()
        }
      }

      override fun isFinished() : Boolean {
        return hasHomed
      }
    }
  }

  fun wristOpenLoopRequest(wristVoltage: ElectricalPotential): Request {
    return object : Request() {
      override fun act() {
        updateCurrentRequest(WristRequests.OPEN_LOOP_REQUEST)
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
        updateCurrentRequest(WristRequests.TARGETING_POSITION)
        executeWristProfile(wristProfile)
      }

      override fun isFinished(): Boolean {
        return isAtTargetedPosition
      }
    }
  }

  fun updateCurrentRequest(other: WristRequests) {
    if (currentWristRequest != other) {
      currentWristRequest = other
    }
  }

  companion object {
    enum class WristRequests {
      UNINITIALIZED,
      HOME,
      TARGETING_POSITION,
      OPEN_LOOP_REQUEST;
    }
  }
}
