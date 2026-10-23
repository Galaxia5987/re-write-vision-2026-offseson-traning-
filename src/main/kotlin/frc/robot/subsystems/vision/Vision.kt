package frc.robot.subsystems.vision

import java.util.Optional
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.linalg.Matrix
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.math.numbers.N1
import org.wpilib.math.numbers.N3

class Vision(val consumer: (Pose2d, Double, Matrix<N3, N1>) -> Unit) {
    private lateinit var curStdDevs: Matrix<N3, N1>
    private val curStdDevsGetter: Matrix<N3, N1>
        get() = curStdDevs

    private val photonCamera: PhotonCamera = PhotonCamera("Camera1")
    private val photonEstimator: PhotonPoseEstimator =
        PhotonPoseEstimator(tagLayout, robotToCam)

    // fun Vision(EstimateConsumer : estConsumer) {
    //     this.estConsumer = estConsumer
    //  }

    fun periodic() {
        var visionEst: Optional<EstimatedRobotPose>
        photonCamera.allUnreadResults.forEach { result ->
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result)
            if (visionEst.isEmpty) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result)
            }
            updateEstimationStdDevs(visionEst, result.getTargets())
            visionEst.ifPresent {
                consumer(
                    it.estimatedPose.toPose2d(),
                    it.timestampSeconds,
                    curStdDevsGetter,
                )
                // var estStdDevs = getEstimatationStdDevs()
            }
        }
    }

    private fun updateEstimationStdDevs(
        estimatedPose: Optional<EstimatedRobotPose>,
        targets: List<PhotonTrackedTarget>,
    ) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = kSingleTagStdDevs
        } else {
            var estStdDevs = kSingleTagStdDevs
            var numTags: Int = 0
            var avgDist: Double = 0.0

            for (tgt in targets) {
                var tagPose =
                    photonEstimator.fieldTags.getTagPose(tgt.getFiducialId())
                if (tagPose.isEmpty()) continue
                numTags++
                avgDist +=
                    tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(
                            estimatedPose
                                .get()
                                .estimatedPose
                                .toPose2d()
                                .getTranslation()
                        )
            }
            if (numTags == 0) {
                curStdDevs = kSingleTagStdDevs
            } else {
                avgDist /= numTags
                if (numTags > 1) {
                    estStdDevs = kMultiTagStdDevs
                }
                if (numTags == 1 && avgDist > 4.0) {
                    estStdDevs =
                        VecBuilder.fill(
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                        )
                } else {
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))
                    curStdDevs = estStdDevs
                }
            }
        }
    }
}
