package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.org.strykeforce.swerve.SwerveDrive;
import frc.lib.org.strykeforce.swerve.SwerveModule;
import frc.robot.utils.WaltonSwerveModule;
import static frc.robot.Constants.SwerveDriveConfig.*;

public class Drivetrain extends SubsystemBase {

    private boolean loadedAzimuthReference = false;

    private final SwerveDrive swerveDrive;
    private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    private final SlewRateLimiter vxRateLimiter;
    private final SlewRateLimiter vyRateLimiter;
    private final SlewRateLimiter vrRateLimiter;

    public Drivetrain() {
        var moduleBuilder =
                new WaltonSwerveModule.Builder()
                        .driveGearRatio(kDriveGearRatio)
                        .wheelDiameterInches(kWheelDiameterInches)
                        .driveMaximumMetersPerSecond(kMaxSpeedMetersPerSecond);

        WaltonSwerveModule[] swerveModules = new WaltonSwerveModule[4];
        Translation2d[] wheelLocations = getWheelLocationMeters();

        vxRateLimiter = new SlewRateLimiter(kMaxSpeedMetersPerSecond);  //max translational velocity rate
        vyRateLimiter = new SlewRateLimiter(kMaxSpeedMetersPerSecond);
        vrRateLimiter = new SlewRateLimiter(kMaxOmega);    //max rotational velocity rate

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

            DutyCycleEncoder encoder = new DutyCycleEncoder(i);
            encoder.setDistancePerRotation(4096.0);

            // TODO: Add trajectory command!

            // TODO: Maybe play with slew rate limiter/linear curve

            // 300 sensor units / 1 ms ->
            // TODO: Add d term
            ProfiledPIDController controller = new ProfiledPIDController(
                    /* 20.0 / 1023.0 */ 20.0 / 4096.0, 0.0, 0.0,
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

//        for (SwerveModule module : getSwerveModules()) {
//            module.storeAzimuthZeroReference();
//        }
    }

    /**
     * Returns the swerve drive kinematics object for use during trajectory configuration.
     *
     * @return the configured kinemetics object
     */
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDrive.getKinematics();
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

        // TODO: Make custom WaltonSwerveDrive class
        if (loadedAzimuthReference) {
            for (SwerveModule module : getSwerveModules()) {
//                ((WaltonSwerveModule) module).setAzimuthRotation2d(Rotation2d.fromDegrees(45));
                ((WaltonSwerveModule) module).periodic();
            }
        }

        SmartDashboard.putNumber("Left front", ((WaltonSwerveModule)getSwerveModules()[0]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Right front", ((WaltonSwerveModule)getSwerveModules()[1]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Left back", ((WaltonSwerveModule)getSwerveModules()[2]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Right back", ((WaltonSwerveModule)getSwerveModules()[3]).getAzimuthAbsoluteEncoderCounts());

        SmartDashboard.putNumber("Left front error", ((WaltonSwerveModule)getSwerveModules()[0]).getAzimuthClosedLoopError());
        SmartDashboard.putNumber("Right front error", ((WaltonSwerveModule)getSwerveModules()[1]).getAzimuthClosedLoopError());
        SmartDashboard.putNumber("Left back error", ((WaltonSwerveModule)getSwerveModules()[2]).getAzimuthClosedLoopError());
        SmartDashboard.putNumber("Right back error", ((WaltonSwerveModule)getSwerveModules()[3]).getAzimuthClosedLoopError());
    }
    //Method necessary for driveRotationVelocityMode translation
    static public Translation2d scaleTranslationInput(Translation2d input) {
        double mag = input.getNorm();
        final double scale = 1.35;
        mag = Math.pow(mag, scale);
        final Rotation2d rotation = new Rotation2d(input.getX(), input.getY());
        return new Translation2d(rotation.getCos() * mag, rotation.getSin() * mag);
    }

    /**
     * Drive the robot with given x, y, and rotational velocities with open-loop velocity control.
     */
    public void drive(
            double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    public void driveRotationVelocityMode(double dx, double dy, double dr) {
        Translation2d translation = Drivetrain.scaleTranslationInput(new Translation2d(dx, dy));

        dx = vxRateLimiter.calculate(translation.getX() * kMaxSpeedMetersPerSecond);
        dy = vyRateLimiter.calculate(translation.getY() * kMaxSpeedMetersPerSecond);
        dr = vrRateLimiter.calculate(dr * kMaxOmega);

        swerveDrive.move(dx, dy, dr, true);
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