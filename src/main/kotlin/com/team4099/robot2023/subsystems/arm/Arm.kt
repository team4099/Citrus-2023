package com.team4099.robot2023.subsystems.arm

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.requests.Request
import com.team4099.robot2023.config.constants.ArmConstants
import com.team4099.robot2023.config.constants.Constants
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Length
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

class Arm(private val io: ArmIO) {
  val inputs = ArmIO.ArmIOInputs()

  private val kP =
    LoggedTunableValue("Arm/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  private val kI =
    LoggedTunableValue(
      "Arm/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Arm/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var prevArmSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  private var armConstraints =
    TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION)

  private var armProfile =
    TrapezoidProfile(
      armConstraints,
      TrapezoidProfile.State(armPositionTarget, 0.degrees.perSecond),
      prevArmSetpoint
    )

  private var armFeedforward: ArmFeedforward

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentState == ArmState.TARGETING_POSITION &&
          armProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
          (inputs.armPosition - armPositionTarget).absoluteValue <= ArmConstants.TOLERANCE
        )

  // Used to make sure that elevator attached to the arm can continue safely
  val canContinueSafely: Boolean
    get() =
      currentRequest is ArmRequest.TargetingPosition &&
        (
          ((Clock.fpgaTime - timeProfileGeneratedAt) - armProfile.totalTime() < 1.0.seconds) ||
            armProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt)
          ) &&
        (inputs.armPosition - armPositionTarget).absoluteValue <= 2.5.degrees

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(ArmConstants.PID.NEO_KP)
      kI.initDefault(ArmConstants.PID.NEO_KI)
      kD.initDefault(ArmConstants.PID.NEO_KD)

      armFeedforward =
        ArmFeedforward(
          ArmConstants.PID.ARM_KS,
          ArmConstants.PID.ARM_KG,
          ArmConstants.PID.ARM_KV,
          ArmConstants.PID.ARM_KA
        )
    } else {
      kP.initDefault(ArmConstants.PID.SIM_KP)
      kI.initDefault(ArmConstants.PID.SIM_KI)
      kD.initDefault(ArmConstants.PID.SIM_KD)

      armFeedforward =
        ArmFeedforward(
          0.0.volts, // Static friction doesn't exist in our simulation
          ArmConstants.PID.ARM_KG,
          ArmConstants.PID.ARM_KV,
          ArmConstants.PID.ARM_KA
        )
    }
  }

  fun generateAndExecuteArmProfile(targetPosition: Length) {}

  fun periodic() {
    io.updateInputs(inputs)

    // Update the PID values when tuning
    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }
  }

  /**
   * Sets the break/idle mode of the arm
   *
   * @param brake The value that break mode for the arm will be set as
   */
  fun setArmBrakeMode(brake: Boolean) {
    io.setArmBrakeMode(brake)
    Logger.getInstance().recordOutput("Arm/armBrakeModeEnabled", brake)
  }

  fun zeroArm() {
    io.zeroEncoder()
  }

  fun regenerateProfileNextLoopCycle() {
    lastArmVoltage = -3337.volts
    lastArmPositionTarget = -3337.degrees
    lastIntakeRunTime = -3337.seconds
  }

  fun setArmVoltage(voltage: ElectricalPotential) {
    //    if ((openLoopForwardLimitReached && voltage > 0.0.volts) ||
    //      (openLoopReverseLimitReached && voltage < 0.0.volts)
    //    ) {
    //      io.setArmVoltage(0.0.volts)
    //    } else {
    io.setArmVoltage(voltage)
    //    }
  }

  /**
   * Sets the arm position using the trapezoidal profile state
   *
   * @param setpoint.position Represents the position the arm should go to
   * @param setpoint.velocity Represents the velocity the arm should be at
   */
  private fun setArmPosition(setpoint: TrapezoidProfile.State<Radian>) {

    // Calculating the acceleration of the arm
    val armAngularAcceleration =
      (setpoint.velocity - prevArmSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevArmSetpoint = setpoint

    // Set up the feed forward variable
    val feedforward =
      armFeedforward.calculate(setpoint.position, setpoint.velocity, armAngularAcceleration)

    // When the forward or reverse limit is reached, set the voltage to 0
    // Else move the arm to the setpoint position
    if (isOutOfBounds(setpoint.velocity)) {
      io.setArmVoltage(armFeedforward.calculate(inputs.armPosition, 0.degrees.perSecond))
    } else {
      io.setArmPosition(setpoint.position, feedforward)
    }

    Logger.getInstance().recordOutput("Arm/profileIsOutOfBounds", isOutOfBounds(setpoint.velocity))
    Logger.getInstance().recordOutput("Arm/armFeedForward", feedforward.inVolts)
    Logger.getInstance().recordOutput("Arm/armTargetPosition", setpoint.position.inDegrees)
    Logger.getInstance().recordOutput("Arm/armTargetVelocity", setpoint.velocity.inDegreesPerSecond)
    Logger.getInstance().recordOutput("Arm/isAtTargetedPosition", isAtTargetedPosition)
  }

  private fun generateArmProfile(): TrapezoidProfile<Radian> {}

  private fun executeArmProfile(armProfile: TrapezoidProfile<Radian>) {}

  private fun isOutOfBounds(velocity: AngularVelocity): Boolean {
    return (velocity > 0.0.degrees.perSecond && forwardLimitReached) ||
      (velocity < 0.0.degrees.perSecond && reverseLimitReached)
  }

  fun setArmAngle(angle: Angle): Request {
    return object : Request() {
      var armProfile: TrapezoidProfile<Radian> = generateArmProfile()

      override fun act() {
        executeArmProfile(armProfile)
      }

      override fun isFinished(): Boolean {
        return isAtTargetedPosition
      }
    }
  }

  companion object {
    enum class ArmState {
      UNINITIALIZED,
      ZEROING_ARM,
      TARGETING_POSITION,
      OPEN_LOOP_REQUEST;

      inline fun equivalentToRequest(request: ArmRequest): Boolean {
        return (
          (request is ArmRequest.OpenLoop && this == OPEN_LOOP_REQUEST) ||
            (request is ArmRequest.TargetingPosition && this == TARGETING_POSITION) ||
            (request is ArmRequest.ZeroArm && this == ZEROING_ARM)
          )
      }
    }
    inline fun fromRequestToState(request: ArmRequest): ArmState {
      return when (request) {
        is ArmRequest.OpenLoop -> ArmState.OPEN_LOOP_REQUEST
        is ArmRequest.TargetingPosition -> ArmState.TARGETING_POSITION
        is ArmRequest.ZeroArm -> ArmState.ZEROING_ARM
      }
    }
  }
}
