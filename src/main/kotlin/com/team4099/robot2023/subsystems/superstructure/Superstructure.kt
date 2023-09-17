package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.requests.EmptyRequest
import com.team4099.lib.requests.Request
import com.team4099.lib.requests.SequentialRequest
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.SuperstructureConstants
import com.team4099.robot2023.subsystems.arm.Arm
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.degrees


class Superstructure(val drivetrain: Drivetrain, val arm: Arm, val elevator: Elevator, val wrist: Wrist) : SubsystemBase() {
  private var activeRequest: Request = EmptyRequest()
  var requestQueue = ArrayList<Request>()
  var hasNewRequest = false
  var allRequestsComplete = false

  private var is_climbing = false
  private val target_location: ControlBoard.ScoringLocation = ControlBoard.ScoringLocation()

  fun clearRequests() {
    requestQueue.clear()
  }
  private fun activateRequest(request: Request) {
    setActiveRequest(request)
    clearRequests()
  }

  private fun setActiveRequest(request: Request) {
    activeRequest = request
    hasNewRequest = true
    allRequestsComplete = false
  }

  fun overrideCurrentRequest() {
    activeRequest = EmptyRequest()
  }

  private fun setRequestQueue(vararg requests: Request) {
    requestQueue = ArrayList(requests.asList())
    hasNewRequest = false
    allRequestsComplete = false
  }

  private fun addRequestToQueue(request: Request) {
    requestQueue.add(request)
  }

  override fun periodic() {
    Logger.getInstance().recordOutput("Superstructure/activeRequest", activeRequest::javaClass.name)

    // Logic when a new request is present and said request isn't an empty request.
    if (hasNewRequest && activeRequest !is EmptyRequest) {
      activeRequest.act()
      hasNewRequest = false
    }

    // Logic when there's no current active request
    if (activeRequest is EmptyRequest) {
      // Check if the queue is empty to define that all requests are complete (nothing left in the
      // queue).
      if (requestQueue.isEmpty()) {
        allRequestsComplete = true
      }
      // If the queue isn't empty, we'd want to take the first request entered into the queue and
      // act on it.
      else {
        activateRequest(requestQueue.removeFirst())
      }
    } else {
      if (activeRequest.isFinished) {
        activeRequest = EmptyRequest()
      } else {
        activeRequest.act()
      }
    }
  }


  fun shelfConeIntake() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.SHELF_CONE_INTAKE
    activateRequest(
      SequentialRequest(
        arm.armClosedLoopRequest(state.armTarget, true),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, false)
      )
    )
  }

  fun shelfCubeIntake() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.SHELF_CUBE_INTAKE
    activateRequest(
      SequentialRequest(
        arm.armClosedLoopRequest(state.armTarget, true),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, false)
      )
    )
  }

  fun stowElevator() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.STOW
    activateRequest(
      SequentialRequest(
        wrist.wristClosedLoopRequest(state.wristTarget, false),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        // elevator.elevatorTuckWaitRequest(0.77)
      )
    )
  }

  fun stowWrist() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.STOW
    activateRequest(
      SequentialRequest(
        wrist.wristClosedLoopRequest(state.wristTarget, true)
      )
    )
  }

  fun stowState() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.STOW
    activateRequest(
      SequentialRequest(
        wrist.wristClosedLoopRequest(state.wristTarget, false),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, true),
        //wrist.wristAboveAngleWait(0.0),
        arm.armClosedLoopRequest(state.armTarget, true)
      )
    )
  }

  fun groundIntakeState() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.GROUND_CONE_INTAKE
    activateRequest(
      SequentialRequest(
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, true),
        //arm.climbRequest(state.armTarget),
        wrist.wristClosedLoopRequest(state.wristTarget, true)
      )
    )
  }

  fun groundIntakeFloatState() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.GROUND_INTAKE_FLOAT
    activateRequest(
      SequentialRequest(
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, true),
        //arm.climbRequest(state.armTarget),
        wrist.wristClosedLoopRequest(state.wristTarget, true)
      )
    )
  }

  fun slideIntakeState() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.SLIDE_INTAKE
    activateRequest(
      SequentialRequest(
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, true),
        arm.armClosedLoopRequest(state.armTarget, true),
        wrist.wristClosedLoopRequest(state.wristTarget, true)
      )
    )
  }

  fun yoshiState() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.YOSHI
    activateRequest(
      SequentialRequest(
        arm.armClosedLoopRequest(state.armTarget, true),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, true)
      )
    )
  }


  // Used only in tele-op
  fun chooseScoreState() {
    when (target_location.level) {
      1 -> scoreL1State()
      2 -> {
        scoreL2State()
      }
      3 -> {
        scoreL3State()
      }
      else -> DriverStation.reportError("Unexpected Scoring State!", false)
    }
  }

  fun dunkState() {
    when (target_location.level) {
      1 -> {}
      2 -> dunkL2State()
      3 -> dunkL3State()
      else -> DriverStation.reportError("Unexpected Scoring State!", false)
    }
  }


  fun scoreStandbyState() {
    scoreStandbyState(false)
  }

  fun scoreStandbyState(force: Boolean) {
    if (!allRequestsComplete && !force) {
      return
    }
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.SCORE_STANDBY
    activateRequest(
      SequentialRequest(
        arm.armClosedLoopRequest(state.armTarget, true),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, true),
        wrist.wristClosedLoopRequest(state.wristTarget, true)
      )
    )
  }

  fun scoreL1State() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.GROUND_SCORE
    activateRequest(
      SequentialRequest(
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, true),
        wrist.wristClosedLoopRequest(state.wristTarget, false),
        arm.armClosedLoopRequest(state.armTarget, false)
      )
    )
  }

  fun scoreL2State() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.L2_SCORE
    activateRequest(
      SequentialRequest(
        arm.armClosedLoopRequest(state.armTarget, true),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, false)
      )
    )
  }

  private fun dunkL2State() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.L2_DUNK
    activateRequest(
      SequentialRequest(
        arm.armClosedLoopRequest(state.armTarget, false),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, true)
      )
    )
  }

  fun scoreL3State() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.L3_SCORE
    activateRequest(
      SequentialRequest(
        arm.armClosedLoopRequest(state.armTarget, true),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        //elevator.elevatorExtendWaitRequest(0.77),
        wrist.wristClosedLoopRequest(state.wristTarget, false)
      )
    )
  }

  private fun dunkL3State() {
    updateClimbStatus(false)
    val state: RobotRequest = SuperstructureConstants.L3_DUNK
    activateRequest(
      SequentialRequest(
        arm.armClosedLoopRequest(state.armTarget, false),
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, true)
      )
    )
  }

  fun climbFloatState() {
    updateClimbStatus(true)
    val state: RobotRequest = SuperstructureConstants.CLIMB_FLOAT
    activateRequest(
      SequentialRequest(
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, false),
        arm.armClosedLoopRequest(state.armTarget, false)
      )
    )
  }

  fun climbScrapeState() {
    updateClimbStatus(true)
    val state: RobotRequest = SuperstructureConstants.CLIMB_SCRAPE
    activateRequest(
      SequentialRequest(
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, false),
        //arm.scrapeRequest(state.armTarget),
      )
    )
  }

  fun climbCurlState() {
    updateClimbStatus(true)
    val state: RobotRequest = SuperstructureConstants.CLIMB_CURL
    activateRequest(
      SequentialRequest(
        elevator.elevatorClosedLoopRequest(state.elevatorTarget, false),
        wrist.wristClosedLoopRequest(state.wristTarget, false),
        //TODO: implement climb
        //arm.climbRequest(state.armTarget)
      )
    )
  }

  // TODO: Finish this
  fun autoBalance() {
    //activateRequest(Drive.getInstance().autoBalanceRequest())
  }

  private fun updateClimbStatus(climb: Boolean) {
    if (is_climbing !== climb) {
      is_climbing = climb
    }
  }
}
