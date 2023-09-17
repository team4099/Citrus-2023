package com.team4099.robot2023.config.constants

import com.team4099.robot2023.subsystems.superstructure.RobotRequest
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees

class SuperstructureConstants {


  val STOW: RobotRequest = RobotRequest(-9.3.degrees, 0.0.meters, 149.5.degrees)

  val GROUND_CONE_INTAKE: RobotRequest = RobotRequest(-9.3.degrees, 0.0.meters, 21.0.degrees)
  val GROUND_CUBE_INTAKE: RobotRequest = RobotRequest(-9.3.degrees, 0.0.meters, 21.0.degrees)

  val GROUND_INTAKE_FLOAT: RobotRequest = RobotRequest(-9.3.degrees, 0.0.meters, 26.0.degrees)

  val SHELF_CONE_INTAKE: RobotRequest = RobotRequest(60.113914.degrees, 0.579607.meters, -50.0.degrees)
  val SHELF_CUBE_INTAKE: RobotRequest = RobotRequest(60.113914.degrees, 0.579607.meters, -50.0.degrees)

  val YOSHI: RobotRequest = RobotRequest(-4.101906.degrees, 0.858025.meters, 13.707724.degrees) // 1.026


  val SLIDE_INTAKE: RobotRequest = RobotRequest(50.0.degrees, 0.00.meters, -6.6.degrees)

  val SCORE_STANDBY: RobotRequest = RobotRequest(39.8.degrees, 0.0.meters, 149.5.degrees)
  val GROUND_SCORE: RobotRequest = RobotRequest(15.0.degrees, 0.0.meters, 15.0.degrees)
  val L2_SCORE: RobotRequest = RobotRequest(39.8.degrees, 0.556.meters, -23.2.degrees)
  val L2_DUNK: RobotRequest = RobotRequest(39.8.degrees, 0.556.meters, -20.2.degrees)
  val L3_SCORE: RobotRequest = RobotRequest(39.8.degrees, 1.04.meters, -17.2.degrees)
  val L3_DUNK: RobotRequest = RobotRequest(39.8.degrees, 1.04.meters, -20.2.degrees)

  val CLIMB_FLOAT: RobotRequest = RobotRequest(100.0.degrees, 0.14.meters, 149.5.degrees)
  val CLIMB_SCRAPE: RobotRequest = RobotRequest(113.0.degrees, 0.14.meters, 149.5.degrees)
  val CLIMB_CURL: RobotRequest = RobotRequest(-9.3.degrees, 0.14.meters, 145.0.degrees)
}
