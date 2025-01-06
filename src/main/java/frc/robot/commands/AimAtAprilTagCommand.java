package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.*;

public class AimAtAprilTagCommand extends Command {
    private final double targetYaw = -5.5;      // Desired yaw value to align to
    private final double yawTolerance = 5.0;    // degrees
    private final double rotationSpeed = 0.15;  // percentage

    final Joystick driverLeftJoystick = new Joystick(0);

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
        if (m_vision.specificAprilTagDetected(aprilTagId)) {
            SmartDashboard.putString("AimAtAprilTag Status", "ACTIVE");
            if ((Math.abs(m_vision.getSpecificAprilTagYaw(aprilTagId)) - Math.abs(targetYaw)) > yawTolerance) {
                if (m_vision.getSpecificAprilTagYaw(aprilTagId) < targetYaw) {
                    // Rotate CCW
                    SmartDashboard.putString("AimAtAprilTag Status", "CCW");
                    m_drive.driveWithSpeeds(-driverLeftJoystick.getY(), -driverLeftJoystick.getX(), rotationSpeed, false); 
                }
                else {
                    // Rotate CW
                    SmartDashboard.putString("AimAtAprilTag Status", "CW");
                    m_drive.driveWithSpeeds(-driverLeftJoystick.getY(), -driverLeftJoystick.getX(), -rotationSpeed, false);
                }
            }
        }
        else {
            SmartDashboard.putString("AimAtAprilTag Status", "APRILTAG NOT SEEN");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AimAtAprilTag Status", "NOT ACTIVE");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
