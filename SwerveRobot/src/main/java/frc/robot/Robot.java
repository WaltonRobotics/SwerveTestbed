// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.RotateModulesToAngle;
import frc.robot.commands.auton.SwerveTrajectoryCommand;
import frc.robot.commands.teleop.DriveCommand;
import frc.robot.commands.teleop.IntakeCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Gamepad.Button;

import static frc.robot.Constants.SmartDashboardKeys.*;
// import static frc.robot.Paths.testTrajectory;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static Intake intake;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        drivetrain = new Drivetrain();
        intake = new Intake();

        CommandScheduler.getInstance().setDefaultCommand(drivetrain, drivetrain.teleopDriveCmd(
                () -> OI.gamepad.getLeftY(), 
                () -> OI.gamepad.getLeftX(), 
                () -> OI.gamepad.getRightX(), 
                () -> OI.gamepad.getButton(Button.LEFT_BUMPER)));
        CommandScheduler.getInstance().setDefaultCommand(intake, new IntakeCommand());

//    SmartDashboard.putData(DRIVETRAIN_SAVE_CURRENT_AZIMUTH_ZERO_KEY,
//            new InstantCommand(drivetrain::saveCurrentPositionsAsAzimuthZeros));

        SmartDashboard.putNumber("Intake voltage", 8);
        SmartDashboard.putData(DRIVETRAIN_ROTATE_MODULES_TO_ANGLE_KEY,
                new RotateModulesToAngle());

        SmartDashboard.putNumber(DRIVETRAIN_SETPOINT_ANGLE_DEGREES, 0.0);

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_AZIMUTH_ZERO_VALUE_KEY, 0.0);
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_AZIMUTH_ZERO_VALUE_KEY, 0.0);
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_AZIMUTH_ZERO_VALUE_KEY, 0.0);
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_AZIMUTH_ZERO_VALUE_KEY, 0.0);

        SmartDashboard.putData(DRIVETRAIN_SAVE_LEFT_FRONT_AZIMUTH_ZERO_KEY,
                new InstantCommand(() ->
                        drivetrain.saveLeftFrontZero((int) SmartDashboard.getNumber(DRIVETRAIN_LEFT_FRONT_AZIMUTH_ZERO_VALUE_KEY, 0.0))));

        SmartDashboard.putData(DRIVETRAIN_SAVE_RIGHT_FRONT_AZIMUTH_ZERO_KEY,
                new InstantCommand(() ->
                        drivetrain.saveRightFrontZero((int) SmartDashboard.getNumber(DRIVETRAIN_RIGHT_FRONT_AZIMUTH_ZERO_VALUE_KEY, 0.0))));

        SmartDashboard.putData(DRIVETRAIN_SAVE_LEFT_REAR_AZIMUTH_ZERO_KEY,
                new InstantCommand(() ->
                        drivetrain.saveLeftRearZero((int) SmartDashboard.getNumber(DRIVETRAIN_LEFT_REAR_AZIMUTH_ZERO_VALUE_KEY, 0.0))));

        SmartDashboard.putData(DRIVETRAIN_SAVE_RIGHT_REAR_AZIMUTH_ZERO_KEY,
                new InstantCommand(() ->
                        drivetrain.saveRightRearZero((int) SmartDashboard.getNumber(DRIVETRAIN_RIGHT_REAR_AZIMUTH_ZERO_VALUE_KEY, 0.0))));

        SmartDashboard.putNumber(DRIVETRAIN_SPEED_MSEC, 0.0);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        drivetrain.reset();
//    drivetrain.loadAzimuthZeroReference();

//    CommandScheduler.getInstance().schedule(new AutoDrive());

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new InstantCommand(() -> intake.setVoltage(.8)),
                // new InstantCommand(() -> drivetrain.resetPose(testTrajectory.getInitialPose())),
                // new SwerveTrajectoryCommand(testTrajectory),
                new InstantCommand(() -> intake.setVoltage(0))
        ));
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
