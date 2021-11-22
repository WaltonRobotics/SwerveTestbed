package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import org.strykeforce.swerve.SwerveModule;

import java.util.logging.Level;

import static com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic;

public class WaltonSwerveModule implements SwerveModule {

    final int k100msPerSecond = 10;

    private final TalonSRX azimuthTalon;
    private final BaseTalon driveTalon;
    private final double azimuthCountsPerRev;
    private final double driveCountsPerRev;
    private final double driveGearRatio;
    private final double wheelCircumferenceMeters;
    private final double driveDeadbandMetersPerSecond;
    private final double driveMaximumMetersPerSecond;
    private final Translation2d wheelLocationMeters;

    private Rotation2d previousAngle = new Rotation2d();

    private WaltonSwerveModule(Builder builder) {
        azimuthTalon = builder.azimuthTalon;
        driveTalon = builder.driveTalon;
        azimuthCountsPerRev = builder.azimuthCountsPerRev;
        driveCountsPerRev = builder.driveCountsPerRev;
        driveGearRatio = builder.driveGearRatio;
        wheelCircumferenceMeters = Math.PI * Units.inchesToMeters(builder.wheelDiameterInches);
        driveDeadbandMetersPerSecond = builder.driveDeadbandMetersPerSecond;
        driveMaximumMetersPerSecond = builder.driveMaximumMetersPerSecond;
        wheelLocationMeters = builder.wheelLocationMeters;
    }

    @Override
    public double getMaxSpeedMetersPerSecond() {
        return driveMaximumMetersPerSecond;
    }

    @Override
    public Translation2d getWheelLocationMeters() {
        return wheelLocationMeters;
    }

    public double getDriveCountsPerRev() {
        return driveCountsPerRev;
    }

    @Override
    public SwerveModuleState getState() {
        double speedMetersPerSecond = getDriveMetersPerSecond();
        Rotation2d angle = getAzimuthRotation2d();
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop) {
        assert desiredState.speedMetersPerSecond >= 0.0;

        if (desiredState.speedMetersPerSecond < driveDeadbandMetersPerSecond) {
            desiredState = new SwerveModuleState(0.0, previousAngle);
        }
        previousAngle = desiredState.angle;

        Rotation2d currentAngle = getAzimuthRotation2d();
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);
        setAzimuthRotation2d(optimizedState.angle);
        if (isDriveOpenLoop) {
            setDriveOpenLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
        } else {
            setDriveClosedLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
        }
    }

    @Override
    public void resetDriveEncoder() {
        var errorCode = driveTalon.setSelectedSensorPosition(0);
        if (errorCode.value != 0) {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Talon error code while resetting encoder to 0: {0}", errorCode);
        }
    }

    @Override
    public void storeAzimuthZeroReference() {
        int index = getWheelIndex();
        int position = getAzimuthAbsoluteEncoderCounts();
        Preferences preferences = Preferences.getInstance();
        String key = String.format("SwerveDrive/wheel.%d", index);
        preferences.putInt(key, position);
        DebuggingLog.getInstance().getLogger().log(Level.INFO, "azimuth {0}: saved zero = {1}", new Object[]{index, position});
    }

    @Override
    public void loadAndSetAzimuthZeroReference() {
        int index = getWheelIndex();
        Preferences preferences = Preferences.getInstance();
        String key = String.format("SwerveDrive/wheel.%d", index);
        int reference = preferences.getInt(key, Integer.MIN_VALUE);
        if (reference == Integer.MIN_VALUE) {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "no saved azimuth zero reference for swerve module {0}", index);
            throw new IllegalStateException();
        }
        DebuggingLog.getInstance().getLogger().log(Level.INFO, "swerve module {0}: loaded azimuth zero reference = {1}", new Object[]{index, reference});

        int azimuthAbsoluteCounts = getAzimuthAbsoluteEncoderCounts();
        DebuggingLog.getInstance().getLogger().log(Level.INFO, "swerve module {0}: azimuth absolute position = {1}", new Object[]{index, azimuthAbsoluteCounts});

        int azimuthSetpoint = azimuthAbsoluteCounts - reference;
        ErrorCode errorCode = azimuthTalon.setSelectedSensorPosition(azimuthSetpoint, 0, 10);
        if (errorCode.value != 0) {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Talon error code while setting azimuth zero: {0}", errorCode);
        }

        azimuthTalon.set(MotionMagic, azimuthSetpoint);
        DebuggingLog.getInstance().getLogger().log(Level.INFO, "swerve module {0}: set azimuth encoder = {1}", new Object[]{index, azimuthSetpoint});
    }

    public TalonSRX getAzimuthTalon() {
        return azimuthTalon;
    }

    public BaseTalon getDriveTalon() {
        return driveTalon;
    }

    private int getAzimuthAbsoluteEncoderCounts() {
        return azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    }

    public Rotation2d getAzimuthRotation2d() {
        double azimuthCounts = azimuthTalon.getSelectedSensorPosition();
        double radians = 2.0 * Math.PI * azimuthCounts / azimuthCountsPerRev;
        return new Rotation2d(radians);
    }

    public void setAzimuthRotation2d(Rotation2d angle) {
        double countsBefore = azimuthTalon.getSelectedSensorPosition();
        double countsFromAngle = angle.getRadians() / (2.0 * Math.PI) * azimuthCountsPerRev;
        double countsDelta = Math.IEEEremainder(countsFromAngle - countsBefore, azimuthCountsPerRev);
        azimuthTalon.set(MotionMagic, countsBefore + countsDelta);
    }

    private double getDriveMetersPerSecond() {
        double encoderCountsPer100ms = driveTalon.getSelectedSensorVelocity();
        double motorRotationsPer100ms = encoderCountsPer100ms / driveCountsPerRev;
        double wheelRotationsPer100ms = motorRotationsPer100ms * driveGearRatio;
        double metersPer100ms = wheelRotationsPer100ms * wheelCircumferenceMeters;
        return metersPer100ms * k100msPerSecond;
    }

    private void setDriveOpenLoopMetersPerSecond(double metersPerSecond) {
        driveTalon.set(ControlMode.PercentOutput, metersPerSecond / driveMaximumMetersPerSecond);
    }

    private void setDriveClosedLoopMetersPerSecond(double metersPerSecond) {
        double wheelRotationsPerSecond = metersPerSecond / wheelCircumferenceMeters;
        double motorRotationsPerSecond = wheelRotationsPerSecond / driveGearRatio;
        double encoderCountsPerSecond = motorRotationsPerSecond * driveCountsPerRev;
        driveTalon.set(ControlMode.Velocity, encoderCountsPerSecond / k100msPerSecond);
    }

    private int getWheelIndex() {
        if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() > 0) {
            return 0;
        }
        if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() < 0) {
            return 1;
        }
        if (wheelLocationMeters.getX() < 0 && wheelLocationMeters.getY() > 0) {
            return 2;
        }
        return 3;
    }

    @Override
    public String toString() {
        return "TalonSwerveModule{" + getWheelIndex() + '}';
    }

    public static class Builder {

        public static final int kDefaultTalonSRXCountsPerRev = 4096;
        public static final int kDefaultTalonFXCountsPerRev = 2048;
        private final int azimuthCountsPerRev = kDefaultTalonSRXCountsPerRev;
        private TalonSRX azimuthTalon;
        private BaseTalon driveTalon;
        private double driveGearRatio;
        private double wheelDiameterInches;
        private int driveCountsPerRev = kDefaultTalonFXCountsPerRev;
        private double driveDeadbandMetersPerSecond = -1.0;
        private double driveMaximumMetersPerSecond;
        private Translation2d wheelLocationMeters;

        public Builder() {}

        public Builder azimuthTalon(TalonSRX azimuthTalon) {
            this.azimuthTalon = azimuthTalon;
            return this;
        }

        public Builder driveTalon(BaseTalon driveTalon) {
            this.driveTalon = driveTalon;
            if (driveTalon instanceof TalonFX) {
                driveCountsPerRev = kDefaultTalonFXCountsPerRev;
                return this;
            }

            if (driveTalon instanceof TalonSRX) {
                driveCountsPerRev = kDefaultTalonSRXCountsPerRev;
                return this;
            }

            throw new IllegalArgumentException("expect drive talon is TalonFX or TalonSRX");
        }

        public Builder driveGearRatio(double ratio) {
            driveGearRatio = ratio;
            return this;
        }

        public Builder wheelDiameterInches(double diameterInches) {
            wheelDiameterInches = diameterInches;
            return this;
        }

        public Builder driveEncoderCountsPerRevolution(int countsPerRev) {
            driveCountsPerRev = countsPerRev;
            return this;
        }

        public Builder driveDeadbandMetersPerSecond(double metersPerSecond) {
            driveDeadbandMetersPerSecond = metersPerSecond;
            return this;
        }

        // we currently only support TalonSRX for azimuth
        //    public Builder azimuthEncoderCountsPerRevolution(int countsPerRev) {
        //      azimuthCountsPerRev = countsPerRev;
        //      return this;
        //    }

        public Builder driveMaximumMetersPerSecond(double metersPerSecond) {
            driveMaximumMetersPerSecond = metersPerSecond;
            return this;
        }

        public Builder wheelLocationMeters(Translation2d locationMeters) {
            wheelLocationMeters = locationMeters;
            return this;
        }

        public WaltonSwerveModule build() {
            if (driveDeadbandMetersPerSecond < 0) {
                driveDeadbandMetersPerSecond = 0.01 * driveMaximumMetersPerSecond;
            }
            var module = new WaltonSwerveModule(this);
            validateWaltonSwerveModuleObject(module);
            return module;
        }

        private void validateWaltonSwerveModuleObject(WaltonSwerveModule module) {
            if (module.azimuthTalon == null) {
                throw new IllegalArgumentException("azimuth talon must be set.");
            }

            if (module.driveGearRatio <= 0) {
                throw new IllegalArgumentException("drive gear ratio must be greater than zero.");
            }

            if (module.azimuthCountsPerRev <= 0) {
                throw new IllegalArgumentException(
                        "azimuth encoder counts per revolution must be greater than zero.");
            }

            if (module.driveCountsPerRev <= 0) {
                throw new IllegalArgumentException(
                        "drive encoder counts per revolution must be greater than zero.");
            }

            if (module.wheelCircumferenceMeters <= 0) {
                throw new IllegalArgumentException("wheel diameter must be greater than zero.");
            }

            if (module.driveMaximumMetersPerSecond <= 0) {
                throw new IllegalArgumentException("drive maximum speed must be greater than zero.");
            }

            if (module.wheelLocationMeters == null) {
                throw new IllegalArgumentException("wheel location must be set.");
            }

            if (module.driveTalon instanceof TalonFX
                    && module.driveCountsPerRev != kDefaultTalonFXCountsPerRev) {
                DebuggingLog.getInstance().getLogger().log(Level.WARNING, "drive TalonFX counts per rev = {0}", module.driveCountsPerRev);
            }

            if (module.driveTalon instanceof TalonSRX
                    && module.driveCountsPerRev != kDefaultTalonSRXCountsPerRev) {
                DebuggingLog.getInstance().getLogger().log(Level.WARNING, "drive TalonSRX counts per rev = {0}", module.driveCountsPerRev);
            }
        }
    }

}
