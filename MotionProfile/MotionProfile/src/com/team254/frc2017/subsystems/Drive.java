package com.team254.frc2017.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2017.Constants;
import com.team254.frc2017.Kinematics;
import com.team254.frc2017.Robot;
import com.team254.frc2017.RobotState;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Util;
import com.team254.lib.util.control.Lookahead;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.control.PathFollower;
import com.team254.lib.util.drivers.CANTalonFactory;
import com.team254.lib.util.drivers.NavX;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Twist2d;

import java.util.Arrays;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons, one solenoid and 2 pistons to shift gears,
 * and a navX board. The Drive subsystem has several control methods including open loop, velocity control, and position
 * control. The Drive subsystem also has several methods that handle automatic aiming, autonomous path driving, and
 * manual control.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    private static final int kLowGearPositionControlSlot = 2;
    private static final int kHighGearVelocityControlSlot = 1;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
        PATH_FOLLOWING, // used for autonomous driving
        
    }

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state) {
        
        return false;
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    public static TalonSRX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final NavX mNavXBoard;

    // Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    private Path mCurrentPath = null;

    // Hardware states
    private boolean mIsBrakeMode;
    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;


    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(0, 0);
                setBrakeMode(false);
                setVelocitySetpoint(0, 0);
                mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
                    return;
                case VELOCITY_SETPOINT:
                    return;
                case PATH_FOLLOWING:
                    if (mPathFollower != null) {
                        updatePathFollower(timestamp);
                    }
                    return;
                
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = CANTalonFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mLeftMaster.setSensorPhase(false);
        mLeftMaster.setInverted(true);
     
        mLeftSlave = CANTalonFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveId,
        		mLeftMaster);
        mLeftSlave.setInverted(false);
        mLeftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);

        mRightMaster = CANTalonFactory.createDefaultTalon(Constants.kRightDriveMasterId);
        mRightMaster.setSensorPhase(false);
        mRightMaster.setInverted(false);
        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
       

        mRightSlave = CANTalonFactory.createPermanentSlaveTalon(Constants.kRightDriverSlaveId,
        		mRightMaster);
        mRightSlave.setInverted(false);
        mRightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);

        mLeftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
        mLeftMaster.configVelocityMeasurementWindow(32, 10);
        mRightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
        mRightMaster.configVelocityMeasurementWindow(32, 10);

        reloadGains();

        setOpenLoop(0, 0);

        // Path Following stuff
        mNavXBoard = new NavX(SPI.Port.kMXP);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(double left, double right) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.configNominalOutputForward(0, 10);
            mLeftMaster.configNominalOutputReverse(0, 10);
            mRightMaster.configNominalOutputForward(0, 10);
	        mRightMaster.configNominalOutputReverse(0, 10);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master.
        mRightMaster.set(ControlMode.PercentOutput, right);
        mLeftMaster.set(ControlMode.PercentOutput, left);
    }


    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            mRightMaster.setNeutralMode(NeutralMode.Coast);
            mRightSlave.setNeutralMode(NeutralMode.Coast);
            mLeftMaster.setNeutralMode(NeutralMode.Coast);
            mLeftSlave.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0, 0);
    }

    @Override
    public void outputToSmartDashboard() {
        final double left_speed = getLeftVelocityInchesPerSec();
        final double right_speed = getRightVelocityInchesPerSec();
        SmartDashboard.putNumber("left voltage (V)", mLeftMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("right voltage (V)", mRightMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("left speed (ips)", left_speed);
        SmartDashboard.putNumber("right speed (ips)", right_speed);
        if (usesTalonVelocityControl(mDriveControlState)) {
            SmartDashboard.putNumber("left speed error (ips)",
                    rpmToInchesPerSecond(mLeftMaster.getClosedLoopError(0)) - left_speed);
            SmartDashboard.putNumber("right speed error (ips)",
                    rpmToInchesPerSecond(mRightMaster.getClosedLoopError(0)) - right_speed);
        } else {
            SmartDashboard.putNumber("left speed error (ips)", 0.0);
            SmartDashboard.putNumber("right speed error (ips)", 0.0);
        }
        synchronized (this) {
            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
                SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
                SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
            } else {
                SmartDashboard.putNumber("drive CTE", 0.0);
                SmartDashboard.putNumber("drive ATE", 0.0);
            }
        }
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
        SmartDashboard.putNumber("gyro pos", getGyroAngle().getDegrees());
        SmartDashboard.putBoolean("drive on target", isOnTarget());
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 10);
        mRightMaster.setSelectedSensorPosition(0, 0, 10);
        
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
        mNavXBoard.zeroYaw();
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(mDriveControlState)) {
            // We entered a velocity control state.
            mLeftMaster.configNominalOutputForward(0, 0);
            mLeftMaster.configNominalOutputReverse(0, 0);
            mLeftMaster.selectProfileSlot(kHighGearVelocityControlSlot, 10);
            mRightMaster.configNominalOutputForward(0, 0);
            mRightMaster.configNominalOutputReverse(0, 0);
            mRightMaster.selectProfileSlot(kHighGearVelocityControlSlot, 10);
            setBrakeMode(true);
        }
    }
     
    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(mDriveControlState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            mLeftMaster.set(ControlMode.Velocity, 1*inchesPerSecondToRpm(left_inches_per_sec * scale));
            mRightMaster.set(ControlMode.Velocity,1* inchesPerSecondToRpm(right_inches_per_sec * scale));
          //  mLeftMaster.set(ControlMode.PercentOutput, Robot.L.getRawAxis(1));
          //  mRightMaster.set(ControlMode.PercentOutput, Robot.R.getRawAxis(1));
            System.out.println(inchesPerSecondToRpm(left_inches_per_sec * scale));
           
        } else {
            System.out.println("Hit a bad velocity control state");
            mLeftMaster.set(ControlMode.PercentOutput, 0);
            mRightMaster.set(ControlMode.PercentOutput, 0);
        }
    }


    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(mLeftMaster.getSelectedSensorPosition(0)* 600/4096);
    }

    public double getRightDistanceInches() {
        return rotationsToInches(mRightMaster.getSelectedSensorPosition(0)* 600/4096);
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(mLeftMaster.getSelectedSensorVelocity(0));
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(mRightMaster.getSelectedSensorVelocity(0));
    }

    public synchronized Rotation2d getGyroAngle() {
        return mNavXBoard.getYaw();
    }

    public synchronized NavX getNavXBoard() {
        return mNavXBoard;
    }

    public synchronized void setGyroAngle(Rotation2d angle) {
        mNavXBoard.reset();
        mNavXBoard.setAngleAdjustment(angle);
    }

    public synchronized double getGyroVelocityDegreesPerSec() {
        return mNavXBoard.getYawRateDegreesPerSec();
    }

    

   
    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isOnTarget() {
        // return true;
        return mIsOnTarget;
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    public boolean isApproaching() {
        return mIsApproaching;
    }



    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.config_kP(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKp, 10);
        mLeftMaster.config_kI(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKi, 10);
        mLeftMaster.config_kD(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKd, 10);
        mLeftMaster.config_kF(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKf, 10);
        mLeftMaster.config_IntegralZone(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionIZone, 10);
        mRightMaster.config_kP(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKp, 10);
        mRightMaster.config_kI(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKi, 10);
        mRightMaster.config_kD(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKd, 10);
        mRightMaster.config_kF(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKf, 10);
        mRightMaster.config_IntegralZone(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionIZone, 10);
        mLeftMaster.config_kP(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKp, 10);
        mLeftMaster.config_kI(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKi, 10);
        mLeftMaster.config_kD(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKd, 10);
        mLeftMaster.config_kF(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKf, 10);
        mLeftMaster.config_IntegralZone(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityIZone, 10);
        mRightMaster.config_kP(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKp, 10);
        mRightMaster.config_kI(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKi, 10);
        mRightMaster.config_kD(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKd, 10);
        mRightMaster.config_kF(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKf, 10);
        mRightMaster.config_IntegralZone(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityIZone, 10);
        mLeftMaster.configOpenloopRamp(Constants.kDriveVoltageCompensationRampRate, 10);
        mLeftMaster.configClosedloopRamp(Constants.kDriveVoltageCompensationRampRate, 10);
        mRightMaster.configOpenloopRamp(Constants.kDriveVoltageCompensationRampRate, 10);
        mRightMaster.configClosedloopRamp(Constants.kDriveVoltageCompensationRampRate, 10);
    }
}
;