package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.org.strykeforce.swerve.SwerveDrive;
import frc.lib.org.strykeforce.swerve.SwerveModule;
import frc.robot.commands.RotateModulesToAngle;
import frc.robot.utils.WaltonSwerveModule;

import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Constants.SwerveDriveConfig.*;

public class Drivetrain extends SubsystemBase {

    private boolean loadedAzimuthReference = false;

    private final SwerveDrive swerveDrive;
    private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    public Drivetrain() {
        var moduleBuilder =
                new WaltonSwerveModule.Builder()
                        .driveGearRatio(kDriveGearRatio)
                        .wheelDiameterInches(kWheelDiameterInches)
                        .driveMaximumMetersPerSecond(kMaxSpeedMetersPerSecond);

        WaltonSwerveModule[] swerveModules = new WaltonSwerveModule[4];
        Translation2d[] wheelLocations = getWheelLocationMeters();

        for (int i = 0; i < 4; i++) {
            var azimuthSparkMax = new CANSparkMax(i + 1, CANSparkMaxLowLevel.MotorType.kBrushless);
            azimuthSparkMax.restoreFactoryDefaults();
            azimuthSparkMax.enableVoltageCompensation(12.0);
            azimuthSparkMax.setSmartCurrentLimit(80);
            azimuthSparkMax.setOpenLoopRampRate(0.0);
            azimuthSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
            azimuthSparkMax.setInverted(true);



            var driveTalon = new TalonFX(i + 11);
            driveTalon.configFactoryDefault(kTalonConfigTimeout);
            driveTalon.configAllSettings(getDriveTalonConfig(), kTalonConfigTimeout);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Brake);

            if (i == 3) {
                driveTalon.setInverted(true);
            }

            DutyCycleEncoder encoder = new DutyCycleEncoder(i);
            encoder.setDistancePerRotation(4096.0);

            // TODO: Add trajectory command!

            // TODO: Maybe play with slew rate limiter/linear curve

            // 300 sensor units / 1 ms ->
            // TODO: Add d term
            ProfiledPIDController controller = new ProfiledPIDController(
                    /* 20.0 / 1023.0 */ 10.0 / 4096.0, 0.0, 0.0,
                    new TrapezoidProfile.Constraints(800 * 10, 1000 * 10)
            );

            controller.setTolerance(30.0);
            controller.enableContinuousInput(-90 * 4096.0, 90 * 4096.0);

            swerveModules[i] =
                    moduleBuilder
                            .azimuthSparkMax(azimuthSparkMax)
                            .driveTalon(driveTalon)
                            .azimuthEncoder(encoder)
                            .azimuthController(controller)
                            .wheelLocationMeters(wheelLocations[i])
                            .build();

            swerveModules[i].loadAndSetAzimuthZeroReference();
            loadedAzimuthReference = true;
        }

        swerveDrive = new SwerveDrive(ahrs, swerveModules);
        resetHeading();
        setHeadingOffset(Rotation2d.fromDegrees(180));

        SmartDashboard.putData(DRIVETRAIN_SAVE_CURRENT_AZIMUTH_ZERO_KEY,
                new InstantCommand(this::saveCurrentPositionsAsAzimuthZeros));

        SmartDashboard.putData(DRIVETRAIN_ROTATE_MODULES_TO_ANGLE_KEY,
                new RotateModulesToAngle());
    }

    public void saveCurrentPositionsAsAzimuthZeros() {
        for (SwerveModule module : getSwerveModules()) {
            module.storeAzimuthZeroReference();
        }
    }

    public void reset() {
        resetDriveEncoders();
        resetHeading();
    }

    /**
     * Returns the swerve drive kinematics object for use during trajectory configuration.
     *
     * @return the configured kinemetics object
     */
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDrive.getKinematics();
    }

    public void resetDriveEncoders() {
        swerveDrive.resetDriveEncoders();
    }

    /**
     * Returns the configured swerve drive modules.
     */
    public SwerveModule[] getSwerveModules() {
        return swerveDrive.getSwerveModules();
    }

    /**
     * Resets the robot's position on the field.
     *
     * @param pose the current pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return the pose of the robot (x and y ane in meters)
     */
    public Pose2d getPoseMeters() {
        return swerveDrive.getPoseMeters();
    }

    /**
     * Perform periodic swerve drive odometry update.
     */
    @Override
    public void periodic() {
        swerveDrive.periodic();

        if (loadedAzimuthReference) {
            for (SwerveModule module : getSwerveModules()) {
                ((WaltonSwerveModule) module).periodic();
            }
        }

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_ABSOLUTE_POSITION, ((WaltonSwerveModule)getSwerveModules()[0]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_ABSOLUTE_POSITION, ((WaltonSwerveModule)getSwerveModules()[1]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_ABSOLUTE_POSITION, ((WaltonSwerveModule)getSwerveModules()[2]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_ABSOLUTE_POSITION, ((WaltonSwerveModule)getSwerveModules()[3]).getAzimuthAbsoluteEncoderCounts());

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_RELATIVE_POSITION, ((WaltonSwerveModule)getSwerveModules()[0]).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_RELATIVE_POSITION, ((WaltonSwerveModule)getSwerveModules()[1]).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_RELATIVE_POSITION, ((WaltonSwerveModule)getSwerveModules()[2]).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_RELATIVE_POSITION, ((WaltonSwerveModule)getSwerveModules()[3]).getAzimuthRelativeEncoderCounts());

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_ANGLE_DEGREES, ((WaltonSwerveModule)getSwerveModules()[0]).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_ANGLE_DEGREES, ((WaltonSwerveModule)getSwerveModules()[1]).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_ANGLE_DEGREES, ((WaltonSwerveModule)getSwerveModules()[2]).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_ANGLE_DEGREES, ((WaltonSwerveModule)getSwerveModules()[3]).getAzimuthRotation2d().getDegrees());

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_VELOCITY_ERROR, ((WaltonSwerveModule)getSwerveModules()[0]).getDriveVelocityError());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_VELOCITY_ERROR, ((WaltonSwerveModule)getSwerveModules()[1]).getDriveVelocityError());
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_VELOCITY_ERROR, ((WaltonSwerveModule)getSwerveModules()[2]).getDriveVelocityError());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_VELOCITY_ERROR, ((WaltonSwerveModule)getSwerveModules()[3]).getDriveVelocityError());
    }

    /**
     * Drive the robot with given x, y, and rotational velocities with open-loop velocity control.
     */
    public void drive(
            double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    /**
     * Move the robot with given x, y, and rotational velocities with closed-loop velocity control.
     */
    public void move(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            boolean isFieldOriented) {
        swerveDrive.move(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, isFieldOriented);
    }

    public void resetHeading() {
        swerveDrive.resetGyro();
    }

    public void setHeadingOffset(Rotation2d offsetRads) {
        swerveDrive.setGyroOffset(offsetRads);
    }

    public Rotation2d getHeading() {
        return swerveDrive.getHeading();
    }

    public void xLockSwerveDrive() {
        ((WaltonSwerveModule) swerveDrive.getSwerveModules()[0])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(45));
        ((WaltonSwerveModule) swerveDrive.getSwerveModules()[1])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
        ((WaltonSwerveModule) swerveDrive.getSwerveModules()[2])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
        ((WaltonSwerveModule) swerveDrive.getSwerveModules()[3])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(45));
    }

}