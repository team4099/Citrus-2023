package com.team4099.robot2023.subsystems.Manipulator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inSeconds
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

  var wristPositionTarget: Angle = 0.0.degrees

  var wristVoltageTarget: ElectricalPotential = 0.0.volts

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  private var lastWristPositionTarget = 0.0.degrees

  private var lastRollerVoltage = 0.0.volts

  var isHomed = false

  var currentState: ManipulatorState = ManipulatorState.UNINITIALIZED
  var currentRequest: Request.ManipulatorRequest = Request.ManipulatorRequest.HomingWrist()
    set(value) {
      when (value) {
        is Request.ManipulatorRequest.OpenLoop -> {
          wristVoltageTarget = value.wristVoltage
          rollerVoltageTarget = value.rollerVoltage
        }
        is Request.ManipulatorRequest.TargetingPosition -> {
          wristPositionTarget = value.wristPosition
          rollerVoltageTarget = value.rollerVoltage
        }
        else -> {}
      }
      field = value
    }

  val forwardLimitReached: Boolean
    get() = inputs.wristPosition >= ManipulatorConstants.ARM_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.wristPosition <= ManipulatorConstants.ARM_MIN_ROTATION

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
      TrapezoidProfile.State(inputs.wristPosition, inputs.wristVelocity)
    )

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var prevWristSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.wristPosition, inputs.wristVelocity)

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

  override fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("Manipulator", inputs)

    Logger.getInstance().recordOutput("Manipulator/currentState", currentState.name)

    Logger.getInstance()
      .recordOutput("Manipulator/requestedState", currentRequest.javaClass.simpleName)

    Logger.getInstance()
      .recordOutput("Manipulator/wristPositionTarget", wristPositionTarget.inDegrees)

    Logger.getInstance().recordOutput("Manipulator/wristVoltageTarget", wristVoltageTarget.inVolts)

    Logger.getInstance()
      .recordOutput("Manipulator/rollerVoltageTarget", rollerVoltageTarget.inVolts)

    Logger.getInstance()
      .recordOutput(
        "Manipulator/isAtCommandedState", currentState.equivalentToRequest(currentRequest)
      )

    Logger.getInstance()
      .recordOutput("Manipulator/lastCommandedAngle", lastWristPositionTarget.inDegrees)

    Logger.getInstance().recordOutput("Manipulator/forwardLimitReached", forwardLimitReached)

    Logger.getInstance().recordOutput("Manipulator/reverseLimitReached", reverseLimitReached)

    var nextState = currentState
    when (currentState) {
      ManipulatorState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      ManipulatorState.HOMING_WRIST -> {
        // Outputs
        if (inputs.statorCurrent < ManipulatorConstants.HOMING_STALL_CURRENT) {
          var lastHomingStatorCurrentTripTime = Clock.fpgaTime
        }
        if (!inputs.isSimulating &&
          (
            !isHomed &&
              inputs.statorCurrent < ManipulatorConstants.HOMING_STALL_CURRENT &&
              (Clock.fpgaTime - lastHomingStatorCurrentTripTime) <
              ManipulatorConstants.HOMING_STALL_TIME_THRESHOLD
            )
        ) {
          setHomeVoltage()
        } else {
          zeroWrist()
          isHomed = true
        }

        // Transition
        if (isHomed) {
          nextState = fromRequestToState(currentRequest)
        }
      }
      ManipulatorState.OPEN_LOOP_REQUEST -> {
        // Outputs
        if (rollerVoltageTarget != lastRollerVoltage) {
          lastIntakeRunTime = Clock.fpgaTime
        }

        setWristVoltage(wristVoltageTarget)
        setRollerVoltage(rollerVoltageTarget)

        // Transitions
        nextState = fromRequestToState(currentRequest)

        // See related comment in targeting position to see why we do this
        if (!(currentState.equivalentToRequest(currentRequest))) {
          lastRollerVoltage = -1337.volts
        }
      }
      ManipulatorState.TARGETING_POSITION -> {
        // Outputs
        if (wristPositionTarget != lastWristPositionTarget) {
          val preProfileGenerate = Clock.realTimestamp
          wristProfile =
            TrapezoidProfile(
              wristConstraints,
              TrapezoidProfile.State(wristPositionTarget, 0.0.degrees.perSecond),
              TrapezoidProfile.State(inputs.wristPosition, 0.0.degrees.perSecond)
            )
          val postProfileGenerate = Clock.realTimestamp
          Logger.getInstance()
            .recordOutput(
              "/Manipulator/ProfileGenerationMS",
              postProfileGenerate.inSeconds - preProfileGenerate.inSeconds
            )
          timeProfileGeneratedAt = Clock.fpgaTime

          // This statement is only run when the wristPositionTarget is first noticed to be
          // different
          // than the previous setpoint the wrist went to.
          lastWristPositionTarget = wristPositionTarget
          lastIntakeRunTime = Clock.fpgaTime
        }

        val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt

        val profileOutput = wristProfile.calculate(timeElapsed)

        setRollerVoltage(rollerVoltageTarget)

        setWristPosition(profileOutput)

        Logger.getInstance()
          .recordOutput(
            "GroundIntake/completedMotionProfile", wristProfile.isFinished(timeElapsed)
          )

        Logger.getInstance()
          .recordOutput("GroundIntake/profilePositionDegrees", profileOutput.position.inDegrees)
        Logger.getInstance()
          .recordOutput(
            "GroundIntake/profileVelocityDegreesPerSecond",
            profileOutput.velocity.inDegreesPerSecond
          )

        // Transitions
        nextState = fromRequestToState(currentRequest)

        // if we're transitioning out of targeting position, we want to make sure the next time we
        // enter targeting position, we regenerate profile (even if the arm setpoint is the same as
        // the previous time we ran it)
        if (!(currentState.equivalentToRequest(currentRequest))) {
          // setting the last target to something unreasonable so the profile is generated next loop
          // cycle
          lastWristPositionTarget = (-1337).degrees
        }
      }
    }

    currentState = nextState
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
    if ((openLoopForwardLimitReached && voltage > 0.0.volts) ||
      (openLoopReverseLimitReached && voltage < 0.0.volts)
    ) {
      io.setWristVoltage(0.0.volts)
    } else {
      io.setWristVoltage(voltage)
    }
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
      io.setWristVoltage(wristFeedforward.calculate(0.0.degrees, 0.degrees.perSecond))
    } else {
      io.setWristPosition(setpoint.position, feedforward)
    }

    Logger.getInstance()
      .recordOutput("Manipulator/wristTargetPosition", setpoint.position.inDegrees)
    Logger.getInstance()
      .recordOutput("Manipulator/wristTargetVelocity", setpoint.velocity.inDegreesPerSecond)
  }

  companion object {
    enum class ManipulatorState {
      UNINITIALIZED,
      OPEN_LOOP_REQUEST,
      TARGETING_POSITION,
      HOMING_WRIST;
      inline fun equivalentToRequest(request: Request.ManipulatorRequest): Boolean {
        return (
          (request is Request.ManipulatorRequest.OpenLoop && this == OPEN_LOOP_REQUEST) ||
            (
              request is Request.ManipulatorRequest.TargetingPosition &&
                this == TARGETING_POSITION
              ) ||
            (request is Request.ManipulatorRequest.HomingWrist && this == HOMING_WRIST)
          )
      }
    }

    inline fun fromRequestToState(request: Request.ManipulatorRequest): ManipulatorState {
      return when (request) {
        is Request.ManipulatorRequest.OpenLoop -> ManipulatorState.OPEN_LOOP_REQUEST
        is Request.ManipulatorRequest.TargetingPosition -> ManipulatorState.TARGETING_POSITION
        is Request.ManipulatorRequest.HomingWrist -> ManipulatorState.HOMING_WRIST
      }
    }
  }
}
