// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.OI;
import frc.robot.Constants.Drive;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        //initilaize slew rate limiters
        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(OI.slewRate);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(OI.slewRate);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * OI.deadband)
                        .withRotationalDeadband(MaxAngularRate * OI.deadband) // Add a deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController xboxController = new CommandXboxController(OI.driverControllerPort);
        public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
                        TunerConstants.DrivetrainConstants,
                        0, // odometry update frequency (0 = use default)
                        VecBuilder.fill(Drive.odometryXYStdDevs, Drive.odometryXYStdDevs, Drive.odometryThetaStdDev),
                        VecBuilder.fill(999, 999, 999), //this is the *default* vision std dev. These values are never used because we always dynamically set it in updateVision()
                        TunerConstants.FrontLeft,
                        TunerConstants.FrontRight,
                        TunerConstants.BackLeft,
                        TunerConstants.BackRight);

        public RobotContainer() {
                configureBindings();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                // Drivetrain default command
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(m_xspeedLimiter.calculate(-xboxController.getLeftY())
                                                                * MaxSpeed)
                                                .withVelocityY(m_yspeedLimiter.calculate(-xboxController.getLeftX())
                                                                * MaxSpeed)
                                                .withRotationalRate(-xboxController.getRightX() * MaxAngularRate)));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                xboxController.b().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-xboxController.getLeftY(),
                                                -xboxController.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                xboxController.back().and(xboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                xboxController.back().and(xboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                xboxController.start().and(xboxController.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                xboxController.start().and(xboxController.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press(LB)
                xboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                //FOR TESTING PURPOSES ONLY; REMOVE/CHANGE FOR COMP: reset position to in front of the center of the red alliance hub, facing red alliance wall(By Apriltags 9 and 10)
                xboxController.leftTrigger().onTrue(
                                new InstantCommand(() -> drivetrain.resetPose(
                                                new Pose2d((492.88 + 15) * 0.0254, (158.32) * 0.0254, //0.0254 converts from in to m
                                                                Rotation2d.fromDegrees(0)))));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}
