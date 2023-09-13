package com.team4099.robot2023.util.CustomSimulation

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.NumericalIntegration
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.MomentOfInertia
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.sin

class WristSim(
  private val gearbox: DCMotor,
  private val gearing: Double,
  private val momentOfInertia: MomentOfInertia,
  private val armLen: Length,
  private val minAngle: Angle,
  private val maxAngle: Angle,
  private val simulateGravity: Boolean,
  private var elevatorAngle: Angle
) : SingleJointedArmSim(
  gearbox,
  gearing,
  momentOfInertia.inKilogramsMeterSquared,
  armLen.inMeters,
  minAngle.inRadians,
  maxAngle.inRadians,
  simulateGravity
) {

  fun updateElevatorAngle(angle: Angle) {
    elevatorAngle = angle
  }


  override fun updateX(
    currentXhat: Matrix<N2?, N1?>?,
    u: Matrix<N1?, N1?>?,
    dtSeconds: Double
  ): Matrix<N2?, N1?>? {
    val updatedXhat = NumericalIntegration.rkdp(
      { x: Matrix<N2?, N1>, _u: Matrix<N1?, N1>? ->
        var xdot =
          m_plant.a.times(x).plus(m_plant.b.times(_u))
        if (simulateGravity) {
          val alphaGrav =
            -14.700000000000001 * (Math.sin(x[0, 0]) * elevatorAngle.sin + Math.cos(x[0, 0]) * elevatorAngle.cos) / armLen.inMeters
          xdot = xdot.plus(VecBuilder.fill(0.0, alphaGrav))
        }
        xdot
      }, currentXhat, u, dtSeconds
    )
    return if (wouldHitLowerLimit(updatedXhat[0, 0])) {
      VecBuilder.fill(minAngle.inRadians, 0.0)
    } else {
      return (if (wouldHitUpperLimit(updatedXhat[0, 0])) VecBuilder.fill(
        maxAngle.inRadians,
        0.0
      ) else updatedXhat) as Matrix<N2?, N1?>
    }
  }

}