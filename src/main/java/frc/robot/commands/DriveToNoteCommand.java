// Robot rotates to align with note, then drives forward until specified area threshold is achieved
// Note camera is in 2D mode

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;

public class DriveToNoteCommand extends Command {
    private final double targetAreaThreshold = 0.20;    // Area threshold to stop driving
    private final double targetYaw = -5.5;              // Desired yaw value to align to
    private final double yawTolerance = 2.0;            // degrees
    private final double driveSpeed = 0.1;              // percentage
    private final double rotationSpeed = 0.1;           // percentage 
    private boolean isAligned = false;                  // Tracks whether alignment is achieved  
    
    private final Drive m_drive;
    private final VisionSubsystem m_vision;

    public DriveToNoteCommand(Drive drive, VisionSubsystem vision) {
        m_drive = drive;
        addRequirements(m_drive);
        m_vision = vision;
        addRequirements(m_vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}    

    @Override
    public void execute() {
        if (m_vision.noteDetected()) {
            // Phase 1: Rotate until aligned
            if (!isAligned) {
                if ((Math.abs(m_vision.getNoteTargetYaw()) - Math.abs(targetYaw)) > yawTolerance) {
                    if (m_vision.getNoteTargetYaw() < targetYaw) {
                        // Rotate CCW
                        SmartDashboard.putString("DriveToNote Status", "CCW");
                        // DriveCommands.joystickDrive(
                        //     m_drive,
                        //     () -> 0,
                        //     () -> 0,
                        //     () -> -rotationSpeed);
                    }
                    else {
                        // Rotate CW
                        SmartDashboard.putString("DriveToNote Status", "CW");
                        // DriveCommands.joystickDrive(
                        //     m_drive,
                        //     () -> 0,
                        //     () -> 0,
                        //     () -> rotationSpeed);
                    }
                }
                else {
                    isAligned = true;   // Alignment achieved
                    // DriveCommands.joystickDrive(
                    //     m_drive,
                    //     () -> 0,
                    //     () -> 0,
                    //     () -> 0);
                }
            }
            else {
                // Phase 2: Drive forward
                if (m_vision.getNoteTargetArea() < targetAreaThreshold) {
                    SmartDashboard.putString("DriveToNote Status", "Drive forward");
                    // DriveCommands.joystickDrive(
                    //     m_drive,
                    //     () -> driveSpeed,
                    //     () -> 0,
                    //     () -> 0);
                }
                else {
                    SmartDashboard.putString("DriveToNote Status", "Finished!");
                }
            }
            

        }
        else {
            SmartDashboard.putString("DriveToNote Status", "NOTE NOT SEEN");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("DriveToNote Status", "NOT ACTIVE");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
