package com.team4099.robot2023.util.CustomSimulation

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.NumericalIntegration
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.MomentOfInertia
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.kilo

class ArmJointSim (
  private val gearbox: DCMotor,
  private val gearing: Double,
  private val armLen: Length,
  private val minAngle: Angle,
  private val maxAngle: Angle,
  private val simulateGravity: Boolean,
  private val initialElevatorExtension: Length
  )
  : SingleJointedArmSim(
  gearbox,
  gearing,
  0.0,
  armLen.inMeters,
  minAngle.inRadians,
  maxAngle.inRadians,
  simulateGravity
    ) {

  private val plant: LinearSystem<N2?, N1?, N1?>
    get() {
      return createSingleJointedArmSystem(gearbox, gearing)
    }


  var elevatorExtension = initialElevatorExtension
    set(value) {
      field = value
    }

  fun getCurrentMomentOfInertia() : MomentOfInertia {
    // line of best fit approximation derived from CAD for J(p) where p is elevator position and J is moment of inertia about arm pivot
    //https://docs.google.com/spreadsheets/d/1xvioK8JDIzYUNmi3L22IS2Vh6wQZYqjEiuNoTYQx_8c/edit?usp=sharing

    return (elevatorExtension.inInches * 5.91 + 4.29).kilo.grams.meterSquared
  }

  fun createSingleJointedArmSystem(
    motor: DCMotor,
    G: Double
  ): LinearSystem<N2?, N1?, N1?> {
    if (G <= 0.0) {
      throw IllegalArgumentException("G must be greater than zero.")
    } else {
      return LinearSystem(Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -Math.pow(G, 2.0) * motor.KtNMPerAmp / (motor.KvRadPerSecPerVolt * motor.rOhms * getCurrentMomentOfInertia().inKilogramsMeterSquared)), VecBuilder.fill(0.0, G * motor.KtNMPerAmp / (motor.rOhms * getCurrentMomentOfInertia().inKilogramsMeterSquared)), Matrix.mat(Nat.N1(), Nat.N2()).fill(1.0, 0.0), Matrix(Nat.N1(), Nat.N1()))
    }
  }

  override fun updateX(
    currentXhat: Matrix<N2?, N1?>?,
    u: Matrix<N1?, N1?>?,
    dtSeconds: Double
  ): Matrix<N2?, N1?>? {
    val updatedXhat = NumericalIntegration.rkdp(
      { x: Matrix<N2?, N1>, _u: Matrix<N1?, N1>? ->
        var xdot =
          plant.a.times(x).plus(plant.b.times(_u))
        if (simulateGravity) {
          val alphaGrav =
            0.5 * -9.8 * Math.cos(x[0, 0]) * armLen.inMeters / getCurrentMomentOfInertia().inKilogramsMeterSquared
          xdot = xdot?.plus(VecBuilder.fill(0.0, alphaGrav))
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
