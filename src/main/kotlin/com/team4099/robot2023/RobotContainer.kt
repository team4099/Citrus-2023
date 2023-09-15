package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.Manipulator.ManipulatorTestCommand
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.Manipulator.Manipulator
import com.team4099.robot2023.subsystems.Manipulator.ManipulatorIONeo
import com.team4099.robot2023.subsystems.Manipulator.ManipulatorIOSim
import com.team4099.robot2023.subsystems.arm.Arm
import com.team4099.robot2023.subsystems.arm.ArmIONeo
import com.team4099.robot2023.subsystems.arm.ArmIOSim
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIOPigeon2
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.elevator.ElevatorIONeo
import com.team4099.robot2023.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2023.util.driver.Ryan
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.Angle
import java.util.function.Supplier

object RobotContainer {
  private val drivetrain: Drivetrain
  private val arm: Arm
  private val elevator: Elevator
  private val manipulator: Manipulator

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      drivetrain = Drivetrain(GyroIOPigeon2, DrivetrainIOReal)
      arm = Arm(ArmIONeo)
      elevator = Elevator(ElevatorIONeo)
      manipulator = Manipulator(ManipulatorIONeo)
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      arm = Arm(ArmIOSim)
      elevator = Elevator(ElevatorIOSim)
      manipulator = Manipulator(ManipulatorIOSim)

      elevator.inputs.elevatorAngle = Supplier<Angle> {arm.inputs.armPosition}
      manipulator.inputs.elevatorAngle = Supplier<Angle> {arm.inputs.armPosition}
    }
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
      TeleopDriveCommand(
        driver = Ryan(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain
      )
  }

  fun requestSuperstructureIdle() {}

  fun regenerateProfiles() {}

  fun zeroSteering() {
    drivetrain.zeroSteering()
  }

  fun zeroSensors() {
    drivetrain.zeroSensors()
    manipulator.zeroWrist()
  }

  fun zeroAngle(toAngle: Angle) {
    drivetrain.zeroGyroYaw(toAngle)
  }

  fun setSteeringCoastMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(false) }
  }
  fun setSteeringBrakeMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(true) }
  }

  fun setDriveCoastMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(false) }
  }

  fun setDriveBrakeMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(true) }
  }

  fun mapTeleopControls() {
    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))
    //    ControlBoard.autoLevel.whileActiveContinuous(
    //      GoToAngle(drivetrain).andThen(AutoLevel(drivetrain))
    //    )
    ControlBoard.manipulatorTest.whileTrue(ManipulatorTestCommand(manipulator))
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)

  fun mapTunableCommands() {}
}
