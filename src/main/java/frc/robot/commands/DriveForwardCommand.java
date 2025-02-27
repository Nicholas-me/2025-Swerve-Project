package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command that drives forward approximately 2 meters using the CTRE Swerve drivetrain.
 */
public class DriveForwardCommand extends SequentialCommandGroup {

    /**
     * Creates a new DriveForwardCommand.
     * 
     * @param drivetrain The drivetrain subsystem to use.
     */
    public DriveForwardCommand(CommandSwerveDrivetrain drivetrain) {
        // Create a simple trajectory-based approach to drive forward
        
        // First, reset the robot's position to ensure accurate movement
        // addCommands(
            // Commands.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d()))
        // );
        
        // Set up the request for driving forward
        SwerveRequest.FieldCentricFacingAngle driveForwardRequest = new SwerveRequest.FieldCentricFacingAngle()
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            // .withDriveRequestType(SwerveRequest.DriveRequestType.OpenLoopVoltage)
            .withVelocityX(2.0) // 2 meters per second in the X direction (forward)
            .withVelocityY(0.0) // No Y velocity
            .withTargetDirection(Rotation2d.fromDegrees(0.0)) // Keep facing forward
            .withDeadband(0.1) // Small deadband for stability
            .withRotationalDeadband(0.1); // Small rotational deadband
            
        // Add the command to drive forward for 1 second
        // Given velocity of 2 m/s, this should move approximately 2 meters
        addCommands(
            drivetrain.applyRequest(() -> driveForwardRequest).withTimeout(1.0)
        );
        
        // Add stop command to ensure the robot comes to a complete stop
        SwerveRequest.FieldCentric stopRequest = new SwerveRequest.FieldCentric()
            // .withDriveRequestType(SwerveRequest.DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0);
            
        addCommands(
            drivetrain.applyRequest(() -> stopRequest).withTimeout(0.1)
        );
    }
}