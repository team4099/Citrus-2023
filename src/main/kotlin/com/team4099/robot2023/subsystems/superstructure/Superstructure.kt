package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.requests.EmptyRequest
import com.team4099.lib.requests.Request
import com.team4099.robot2023.subsystems.arm.Arm
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Superstructure(arm: Arm) : SubsystemBase() {
  var activeRequest: Request = EmptyRequest()
  var requestQueue = ArrayList<Request>()
  var hasNewRequest = false
  var allRequestsComplete = false

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
}
