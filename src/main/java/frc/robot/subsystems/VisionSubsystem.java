package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase{
    // Create PhotonCamera objects
    // TODO: Change camera name in settings to match
    private final PhotonCamera aprilTagCamera = new PhotonCamera("AprilTagCamera");
    // TODO: Implement once aprilTagCamera fully works
    // private final PhotonCamera noteCamera = new PhotonCamera("NoteCamera");

    // Initialize variables
    private boolean aprilTagHasTargets;
    private int aprilTagTargetId;
    private double aprilTagTargetYaw;
    private double aprilTagTargetPitch;
    private double aprilTagTargetArea;

    public VisionSubsystem() {
        // Additional initialization if needed
    }

    @Override
    public void periodic() {
        // Read in relevant data from the Camera
        var aprilTagResults = aprilTagCamera.getAllUnreadResults();
        if (!aprilTagResults.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var aprilTagResult = aprilTagResults.get(aprilTagResults.size() - 1);

            if (aprilTagResult.hasTargets()) {
                // At least one AprilTag was seen by the camera
                aprilTagHasTargets = true;
                // Gets best target (for multiple targets)
                aprilTagTargetId = aprilTagResult.getBestTarget().getFiducialId();
                aprilTagTargetYaw = aprilTagResult.getBestTarget().getYaw();
                aprilTagTargetPitch = aprilTagResult.getBestTarget().getPitch();
                aprilTagTargetArea = aprilTagResult.getBestTarget().getArea();
            }
            else {
                aprilTagHasTargets = false;
                aprilTagTargetId = 999;
                aprilTagTargetYaw = 0;
                aprilTagTargetPitch = 0;
                aprilTagTargetArea = 0;
            }

        // Publish to SmartDashboard
        SmartDashboard.putBoolean("AprilTag Dectected?", aprilTagHasTargets);
        SmartDashboard.putNumber("AprilTag ID", aprilTagTargetId);
        SmartDashboard.putNumber("AprilTag Yaw", aprilTagTargetYaw);
        SmartDashboard.putNumber("AprilTag Pitch", aprilTagTargetPitch);
        SmartDashboard.putNumber("AprilTag Area", aprilTagTargetArea);
        
        
        // Skew for notes???
        // aprilTagTargetSkew = aprilTagResult.getBestTarget().getSkew();
        }
    }

    // Getter methods to access target data
    public boolean aprilTagDectected() {
        return aprilTagHasTargets;
    }

    // public boolean hasValidTargets(PhotonPipelineResult result) {
    //     return result != null && result.hasTargets();
    // }

    // public double getBestTargetYaw(PhotonPipelineResult result) {
    //     if (hasValidTargets(result)) {
    //         return result.getBestTarget().getYaw();
    //     }
    //     return 0.0; // Default value if no targets
    // }
}
