package com.team4099.robot2023.auto

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand

object AutonomousSelector {

  fun getCommand(drivetrain: Drivetrain): CommandBase {
    return InstantCommand()
  }
}
