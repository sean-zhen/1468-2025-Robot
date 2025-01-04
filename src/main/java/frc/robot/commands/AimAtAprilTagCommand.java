package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.*;

public class AimAtAprilTagCommand extends Command {
    private final Drive m_drive;
    private final VisionSubsystem m_vision;
    private final int aprilTagId;

    public AimAtAprilTagCommand(Drive drive, VisionSubsystem vision, int id) {
        m_drive = drive;
        addRequirements(m_drive);
        m_vision = vision;
        addRequirements(m_vision);
        aprilTagId = id;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_vision.aprilTagDectected()) {
            SmartDashboard.putString("LockToAprilTag Status", "ACTIVE");
        }
        else {
            SmartDashboard.putString("LockToAprilTag Status", "APRILTAG NOT SEEN");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("LockToAprilTag Status", "NOT ACTIVE");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
