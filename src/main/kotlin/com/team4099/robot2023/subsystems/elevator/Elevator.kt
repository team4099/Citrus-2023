package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.requests.Request
import com.team4099.robot2023.config.constants.ArmConstants
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.arm.Arm
import com.team4099.robot2023.util.CustomFeedForward.PivotElevatorFeedForward
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import org.team4099.lib.units.perSecond

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  val inputs = ElevatorIO.ElevatorInputs()

  private var elevatorFeedforwardFirstStage: PivotElevatorFeedForward =
    PivotElevatorFeedForward(
      ElevatorConstants.REAL_ELEVATOR_KS_FIRST_STAGE,
      ElevatorConstants.ELEVATOR_KG_FIRST_STAGE,
      ElevatorConstants.ELEVATOR_KV_FIRST_STAGE,
      ElevatorConstants.ELEVATOR_KA_FIRST_STAGE,
      ArmConstants.MIN_ROTATION
    )
  private var elevatorFeedforwardSecondStage: PivotElevatorFeedForward =
    PivotElevatorFeedForward(
      ElevatorConstants.REAL_ELEVATOR_KS_SECOND_STAGE,
      ElevatorConstants.ELEVATOR_KG_SECOND_STAGE,
      ElevatorConstants.ELEVATOR_KV_SECOND_STAGE,
      ElevatorConstants.ELEVATOR_KA_SECOND_STAGE,
      ArmConstants.MIN_ROTATION
    )

  // PID and Feedforward Values
  var elevatorFeedforward: PivotElevatorFeedForward = elevatorFeedforwardFirstStage
    get() {
      return if (inputs.elevatorPosition > ElevatorConstants.FIRST_STAGE_HEIGHT) {
        elevatorFeedforwardSecondStage
      } else {
        elevatorFeedforwardFirstStage
      }
    }

  private var currentElevatorRequest = ElevatorRequests.UNINITIALIZED

  private val kP =
    LoggedTunableValue("Elevator/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  private val kI =
    LoggedTunableValue(
      "Elevator/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Elevator/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  object TunableElevatorHeights {
    val enableElevator =
      LoggedTunableNumber("Elevator/enableMovementElevator", ElevatorConstants.ENABLE_ELEVATOR)

    val minPosition =
      LoggedTunableValue(
        "Elevator/minPosition",
        ElevatorConstants.ELEVATOR_IDLE_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val maxPosition =
      LoggedTunableValue(
        "Elevator/maxPosition",
        ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val openLoopExtendVoltage =
      LoggedTunableValue(
        "Elevator/openLoopExtendVoltage", 8.volts, Pair({ it.inVolts }, { it.volts })
      )

    val openLoopRetractVoltage =
      LoggedTunableValue(
        "Elevator/openLoopRetractVoltage", (-12.0).volts, Pair({ it.inVolts }, { it.volts })
      )
  }

  val forwardLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_SOFT_LIMIT_RETRACTION

  val forwardOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFTLIMIT_EXTENSION
  val reverseOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFTLIMIT_RETRACTION

  val isStowed: Boolean
    get() =
      inputs.elevatorPosition <=
        TunableElevatorHeights.minPosition.get() + ElevatorConstants.ELEVATOR_TOLERANCE

  var isHomed = false

  var elevatorPositionTarget = 0.0.inches
    private set

  private var lastRequestedPosition = -1337.inches

  private var lastRequestedVelocity = -1337.inches.perSecond

  private var lastRequestedVoltage = -1337.volts

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var lastHomingStatorCurrentTripTime = -1337.seconds

  // trapezoidal profile constraints
  private var elevatorConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
    )

  private var elevatorSetpoint: TrapezoidProfile.State<Meter> =
    TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)

  private var elevatorProfile =
    TrapezoidProfile(
      elevatorConstraints,
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond),
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond)
    )

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentElevatorRequest == ElevatorRequests.TARGETING_POSITION &&
          elevatorProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
          (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
          ElevatorConstants.ELEVATOR_TOLERANCE
        ) ||
        (TunableElevatorHeights.enableElevator.get() != 1.0)

  val canContinueSafely: Boolean
    get() =
      currentElevatorRequest == ElevatorRequests.TARGETING_POSITION &&
        (
          ((inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <= 5.inches) ||
            elevatorProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt)
          ) &&
        lastRequestedPosition == elevatorPositionTarget

  init {
    TunableElevatorHeights

    // initializing pid constants and changing FF for sim vs real
    if (RobotBase.isReal()) {
      isHomed = false

      kP.initDefault(ElevatorConstants.REAL_KP)
      kI.initDefault(ElevatorConstants.REAL_KI)
      kD.initDefault(ElevatorConstants.REAL_KD)

      elevatorFeedforwardSecondStage =
        PivotElevatorFeedForward(
          ElevatorConstants.REAL_ELEVATOR_KS_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KG_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KV_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KA_SECOND_STAGE,
          ArmConstants.MIN_ROTATION

        )

      elevatorFeedforwardFirstStage =
        PivotElevatorFeedForward(
          ElevatorConstants.REAL_ELEVATOR_KS_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KG_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KV_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KA_FIRST_STAGE,
          ArmConstants.MIN_ROTATION
        )
    } else {
      isHomed = true

      kP.initDefault(ElevatorConstants.SIM_KP)
      kI.initDefault(ElevatorConstants.SIM_KI)
      kD.initDefault(ElevatorConstants.SIM_KD)

      elevatorFeedforwardSecondStage =
        PivotElevatorFeedForward(
          ElevatorConstants.REAL_ELEVATOR_KS_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KG_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KV_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KA_SECOND_STAGE,
          ArmConstants.MIN_ROTATION
        )

      elevatorFeedforwardFirstStage =
        PivotElevatorFeedForward(
          ElevatorConstants.REAL_ELEVATOR_KS_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KG_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KV_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KA_FIRST_STAGE,
          ArmConstants.MIN_ROTATION
        )

      io.configPID(kP.get(), kI.get(), kD.get())
    }
  }

  override fun periodic() {
    io.updateInputs(inputs)

    elevatorFeedforward.elevatorAngle = inputs.elevatorAngle.get()

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("Elevator", inputs)
    Logger.getInstance()
      .recordOutput("Elevator/currentRequest", currentElevatorRequest.name)

    Logger.getInstance().recordOutput("Elevator/isAtTargetedPosition" , isAtTargetedPosition)
  }

  /**
   * Sets the voltage of the elevator motors but also checks to make sure elevator doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setOutputVoltage(voltage: ElectricalPotential) {
    if (forwardLimitReached && voltage > 0.volts || reverseLimitReached && voltage < 0.volts) {
      io.setOutputVoltage(0.volts)
    } else {
      io.setOutputVoltage(voltage)
    }
  }

  private fun setHomeVoltage(voltage: ElectricalPotential) {
    io.setOutputVoltage(voltage)
  }



  /**
   * Sets the elevator to a specific position using trapezoidal profile state and feedforward also
   * has safety for max extension and retractions
   *
   * @param voltage the voltage to set the motor to
   */
  fun setElevatorPosition(setpoint: TrapezoidProfile.State<Meter>) {
    val elevatorAccel =
      ((setpoint.velocity - elevatorSetpoint.velocity) / (Constants.Universal.LOOP_PERIOD_TIME))

    elevatorSetpoint = setpoint

    val feedforward = elevatorFeedforward.calculate(setpoint.velocity, elevatorAccel)

    if (forwardLimitReached && setpoint.position > inputs.elevatorPosition ||
      reverseLimitReached && setpoint.position < inputs.elevatorPosition
    ) {
      io.setOutputVoltage(0.0.volts)
    } else {
      io.setPosition(setpoint.position, feedforward)
    }

    Logger.getInstance().recordOutput("Elevator/targetPosition", setpoint.position.inInches)
    Logger.getInstance().recordOutput("Elevator/targetVel", setpoint.velocity.inInchesPerSecond)
    Logger.getInstance()
      .recordOutput("Elevator/elevatorAcceleration", elevatorAccel.inInchesPerSecondPerSecond)
    Logger.getInstance().recordOutput("Elevator/elevatorFeedForward", feedforward.inVolts)
  }

  private fun generateElevatorProfile(): TrapezoidProfile<Meter> {
    val preProfileGenerate = Clock.realTimestamp
    elevatorProfile =
      TrapezoidProfile(
        elevatorConstraints,
        TrapezoidProfile.State(elevatorPositionTarget, 0.0.meters.perSecond),
        TrapezoidProfile.State(inputs.elevatorPosition, 0.0.meters.perSecond)
      )
    val postProfileGenerate = Clock.realTimestamp

    timeProfileGeneratedAt = Clock.fpgaTime

    Logger.getInstance().recordOutput("Elevator/ProfileGenerationTime", (postProfileGenerate - preProfileGenerate).inSeconds)
    return elevatorProfile
  }

  private fun executeElevatorProfile(elevatorProfile: TrapezoidProfile<Meter>) {
    val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt
    val profileOutput = elevatorProfile.calculate(timeElapsed)
    setElevatorPosition(profileOutput)

    Logger.getInstance()
      .recordOutput(
        "Elevator/completedMotionProfile", elevatorProfile.isFinished(timeElapsed)
      )

    Logger.getInstance()
      .recordOutput("Elevator/profilePositionInches", profileOutput.position.inInches)
    Logger.getInstance()
      .recordOutput(
        "Elevator/profileVelocityInchesPerSecond",
        profileOutput.velocity.inInchesPerSecond
      )
  }

  private fun isOutOfBounds(velocity: LinearVelocity): Boolean {
    return (velocity > 0.0.meters.perSecond && forwardLimitReached) ||
      (velocity < 0.0.meters.perSecond && reverseLimitReached)
  }

  /** set the current encoder position to be the encoders zero value */
  fun zeroEncoder() {
    io.zeroEncoder()
  }

  fun isAtHome() : Boolean {
    if (inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STALL_CURRENT) {
      lastHomingStatorCurrentTripTime = Clock.fpgaTime
    }
    return (inputs.isSimulating ||
      (
        isHomed &&
          inputs.leaderStatorCurrent >= ElevatorConstants.HOMING_STALL_CURRENT &&
          (Clock.fpgaTime - lastHomingStatorCurrentTripTime) >=
          ElevatorConstants.HOMING_STALL_TIME_THRESHOLD
      )
    )
  }



  fun elevatorHomeRequest(): Request {
    return object : Request() {
      var hasHomed = false

      override fun act() {
        updateCurrentRequest(ElevatorRequests.HOME)
        if (isAtHome()) {
          hasHomed = true
          zeroEncoder()
        } else {
          setHomeVoltage(ElevatorConstants.HOMING_APPLIED_VOLTAGE)
        }
      }

      override fun isFinished(): Boolean {
        return hasHomed
      }
    }
  }

  fun elevatorOpenLoopRequest(elevatorVoltage: ElectricalPotential): Request {
    return object : Request() {
      override fun act() {
        updateCurrentRequest(ElevatorRequests.OPEN_LOOP)
        setOutputVoltage(elevatorVoltage)
      }

      override fun isFinished(): Boolean {
        return false
      }
    }
  }

  fun elevatorClosedLoopRequest(position: Length, waitForPosition: Boolean): Request {
    return object : Request() {
      var armProfile: TrapezoidProfile<Meter> = generateElevatorProfile()

      init {
        elevatorPositionTarget = position
      }

      override fun act() {
        updateCurrentRequest(ElevatorRequests.TARGETING_POSITION)
        executeElevatorProfile(elevatorProfile)
      }

      override fun isFinished(): Boolean {
        return isAtTargetedPosition
      }
    }
  }

  fun updateCurrentRequest(other: ElevatorRequests) {
    if (currentElevatorRequest != other) {
      currentElevatorRequest = other
    }
  }


  companion object {
    enum class ElevatorRequests {
      UNINITIALIZED,
      TARGETING_POSITION,
      OPEN_LOOP,
      HOME;
    }
  }
}
