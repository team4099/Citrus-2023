package com.team4099.robot2023.commands.elevator

import com.team4099.lib.requests.EmptyRequest
import com.team4099.lib.requests.Request
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.degrees

class elevatorTestCommand(val elevator: Elevator) : CommandBase() {
  var request: Request = EmptyRequest()

  init {
    addRequirements(elevator)
  }

  override fun initialize() {
    request = elevator.elevatorClosedLoopRequest(30.inches, false)
  }

  override fun execute() {
    request.act()
  }

  override fun isFinished(): Boolean {
    return false
  }
}
