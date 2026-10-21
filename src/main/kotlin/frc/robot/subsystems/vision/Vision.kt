package frc.robot.subsystems.vision

import org.ejml.data.DMatrix3
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.wpilib.math.geometry.Pose2d
import java.util.Optional
import org.photonvision.EstimatedRobotPose
import org.photonvision.targeting.PhotonTrackedTarget

class Vision(val consumer: (Pose2d, Double, DMatrix3) -> Unit) {
    private lateinit var curStdDevs: DMatrix3
    private val curStdDevsGetter: DMatrix3 get() = curStdDevs



    private val photonCamera: PhotonCamera = PhotonCamera("Camera1")
    private val photonEstimator: PhotonPoseEstimator = PhotonPoseEstimator(tagLayout, robotToCam)

    // val matrix<N3, N1> kSingleTagStdDevs = VecBuild.fill(4,4,8)

    fun periodic() {
        var visionEst:Optional<EstimatedRobotPose>
        photonCamera.allUnreadResults.forEach { result ->
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result)
            if (visionEst.isEmpty) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result)
            }
            updateEstimationStdDevs(visionEst, result.getTargets())
            visionEst.ifPresent {
                consumer(it.estimatedPose.toPose2d(), it.timestampSeconds, curStdDevsGetter)
                // var estStdDevs = getEstimatationStdDevs()
            }
        }
    }

    private fun updateEstimationStdDevs(
        visionEst: Optional<EstimatedRobotPose>, targets: List<PhotonTrackedTarget>) {
       // if (estimatedPose.isEmpty()){
       // curStdDevs = kSingleTagStdDevs
        }
    }




