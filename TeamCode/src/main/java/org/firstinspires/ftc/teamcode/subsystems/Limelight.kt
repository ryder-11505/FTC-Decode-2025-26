package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.LLStatus
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

/**
 * A helper class to manage Limelight3A vision without being tied to an OpMode.
 * Call [update] inside your loop to refresh data.
 */
class Limelight(hardwareMap: HardwareMap, deviceName: String) {

    fun interface IntProvider {
        fun run(): Int
    }

    private val limelight: Limelight3A = hardwareMap.get(Limelight3A::class.java, deviceName)

    var latestStatus: LLStatus? = null
        private set

    var latestResult: LLResult? = null
        private set

    // Target ID provider (configurable at runtime)
    private var targetIdProvider: IntProvider? = null

    fun setTargetIdProvider(provider: IntProvider) {
        targetIdProvider = provider
    }

    /** The currently requested target ID (or null if none set) */
    val targetId: Int?
        get() = targetIdProvider?.run()

    /** The AprilTag detection that matches the targetId (null if none found) */
    val trackedTag: LLResultTypes.FiducialResult?
        get() {
            val id = targetId ?: return null
            return latestResult?.fiducialResults?.firstOrNull { it.fiducialId == id }
        }

    init {
        limelight.pipelineSwitch(0) // default pipeline
        limelight.start()           // start polling for data
        limelight.setPollRateHz(100)
    }

    /** Refresh Limelight data, call once per loop */
    fun update() {
        latestStatus = limelight.status
        latestResult = limelight.latestResult
    }

    fun start() {
        limelight.pipelineSwitch(0) // default pipeline
        limelight.start()           // start polling for data
        limelight.setPollRateHz(100)
    }

    /** Get robot pose (null if no valid result) */
    val botPose: Pose3D?
        get() = latestResult?.botpose

    /** Get fiducial detections (AprilTags, etc.) */
    val fiducials: List<LLResultTypes.FiducialResult>?
        get() = latestResult?.fiducialResults

    /** Horizontal offset from tracked tag */
    val tx: Double?
        get() = trackedTag?.targetXDegrees

    /** Vertical offset from tracked tag */
    val ty: Double?
        get() = trackedTag?.targetYDegrees

    /** Target area (optional size metric) */
    val ta: Double?
        get() = trackedTag?.targetArea

    /** Stop Limelight polling */
    fun stop() {
        limelight.stop()
    }

    val txnc: Double?
        get() = latestResult?.takeIf { it.isValid }?.txNC

    val tync: Double?
        get() = latestResult?.takeIf { it.isValid }?.tyNC

    val latency: Double?
        get() = latestResult?.let { it.captureLatency + it.targetingLatency }

    val parseLatency: Double?
        get() = latestResult?.parseLatency

    val horizontalDistance: Double?
        get() = trackedTag?.robotPoseTargetSpace?.position?.y

    val pythonOutput: DoubleArray?
        get() = latestResult?.pythonOutput

    /** Example: get details from each detected fiducial */
    fun getFiducialData(): List<Map<String, Any?>> {
        val results = fiducials ?: return emptyList()
        return results.map { fr ->
            mapOf(
                "id" to fr.fiducialId,
                "family" to fr.family,
                "xDeg" to fr.targetXDegrees,
                "yDeg" to fr.targetYDegrees,
                "cameraPose" to fr.cameraPoseTargetSpace,
                "robotPoseField" to fr.robotPoseFieldSpace,
                "tagPose" to fr.targetPoseCameraSpace,
                "strafeDistance" to fr.robotPoseTargetSpace.position.y
            )
        }
    }
}