package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TagCenteringCommand extends Command {
    private final RobotContainer robotContainer;
    private final DriveSubsystem driveSubsystem;
    private final PhotonCamera aprilTagCamera;
    private final RunCommand defaultDriveCommand;

    private Field2d field;

    public TagCenteringCommand(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.driveSubsystem = robotContainer.m_robotDrive;
        this.aprilTagCamera = robotContainer.m_aprilTagCamera;
        this.defaultDriveCommand = robotContainer.m_defaultDriveCommand;

        this.driveSubsystem.setDefaultCommand(null);

        this.field = new Field2d();
        SmartDashboard.putData("Field", this.field);

        addRequirements(this.driveSubsystem);
    }

    @Override 
    public void initialize() {
        PhotonPipelineResult result = this.aprilTagCamera.getLatestResult();

        if (!result.hasTargets()) {
            this.end(true);
            return;
        }

        PhotonTrackedTarget target = result.getBestTarget();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
