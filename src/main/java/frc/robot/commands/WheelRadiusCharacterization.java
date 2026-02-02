package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.Drive;
import com.ctre.phoenix6.swerve.SwerveRequest;
import java.util.function.DoubleSupplier;

public class WheelRadiusCharacterization extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric spinRequest = new SwerveRequest.RobotCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(1.0); // Increased slightly for efficiency, adjust as needed

    private double lastYaw;
    private double accumulatedYaw = 0;
    private final double[] lastWheelPos = new double[4];
    private double accumulatedWheelRot = 0;

    // Calculate drivebase radius (hypotenuse from center to module)
    private final double driveBaseRadius = Math.hypot(Drive.wheelXtocenter, Drive.wheelYtocenter);

    public WheelRadiusCharacterization(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        accumulatedYaw = 0;
        accumulatedWheelRot = 0;

        // Get initial gyro state (in radians)
        lastYaw = drivetrain.getState().Pose.getRotation().getRadians();

        // Get initial wheel positions (in Rotations)
        for (int i = 0; i < 4; i++) {
            lastWheelPos[i] = drivetrain.getModule(i).getDriveMotor().getPosition().getValueAsDouble();
        }

        System.out.println("=== Wheel Radius Characterization START ===");
    }

    @Override
    public void execute() {
        drivetrain.setControl(spinRequest);

        // ---- Robot Rotation (Gyro) ----
        double currentYaw = drivetrain.getState().Pose.getRotation().getRadians();
        double deltaYaw = currentYaw - lastYaw;

        // Handle Gyro wrapping (-PI to PI)
        if (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;
        if (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;

        accumulatedYaw += Math.abs(deltaYaw);
        lastYaw = currentYaw;

        // ---- Wheel Rotation (Encoders) ----
        double wheelDeltaSum = 0;
        for (int i = 0; i < 4; i++) {
            double pos = drivetrain.getModule(i).getDriveMotor().getPosition().getValueAsDouble();
            double delta = pos - lastWheelPos[i];
            lastWheelPos[i] = pos;
            wheelDeltaSum += Math.abs(delta);
        }
        
        // Average the 4 modules
        accumulatedWheelRot += wheelDeltaSum / 4.0;
    }
    @Override
    public boolean isFinished() {
        //ends the command once the robot has spun 10 full circles
        return accumulatedYaw >= (2 * Math.PI * 10);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds());

        if (accumulatedWheelRot > 0) {
            // Formula: Radius = (RobotArcLength) / (WheelRadians)
            // RobotArcLength = RobotRadians * DriveBaseRadius
            // WheelRadians = WheelRotations * 2PI
            double wheelRadiusMeters = (accumulatedYaw * driveBaseRadius) / (accumulatedWheelRot * 2 * Math.PI);
            double wheelRadiusInches = wheelRadiusMeters * 39.3701;

            System.out.println("=== Characterization DONE ===");
            System.out.printf("Total Robot Rotation: %.2f rads%n", accumulatedYaw);
            System.out.printf("Avg Wheel Rotation:   %.2f rots%n", accumulatedWheelRot);
            System.out.printf("Wheel Radius Estimate: %.5f inches (%.5f m)%n", wheelRadiusInches, wheelRadiusMeters);
        } else {
            System.out.println("=== Characterization FAILED: No movement detected ===");
        }
    }
}