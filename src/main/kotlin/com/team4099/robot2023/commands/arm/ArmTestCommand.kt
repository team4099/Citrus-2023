package com.team4099.robot2023.commands.arm

import com.team4099.lib.requests.EmptyRequest
import com.team4099.lib.requests.Request
import com.team4099.robot2023.subsystems.arm.Arm
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.degrees

class ArmTestCommand(val arm: Arm, val elevator: Elevator) : CommandBase() {
  var armRequest: Request = EmptyRequest()
  var elevatorRequest: Request = EmptyRequest()

  init {
    addRequirements(arm)
  }

  override fun initialize() {
    armRequest = arm.armClosedLoopRequest(60.degrees, false)
    elevatorRequest = elevator.elevatorClosedLoopRequest(20.inches, false)
  }

  override fun execute() {
    armRequest.act()
    elevatorRequest.act()
  }

  override fun isFinished(): Boolean {
    return false
  }
}
