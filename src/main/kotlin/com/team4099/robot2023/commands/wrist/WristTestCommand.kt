package com.team4099.robot2023.commands.wrist

import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.CommandBase

class WristTestCommand(val wrist: Wrist) : CommandBase() {
  init {
    addRequirements(wrist)
  }

  override fun execute() {
  }

  override fun end(interupted: Boolean) {
  }
}
