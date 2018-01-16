package com.team254.frc2017;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.frc2017.auto.AutoModeExecuter;
import com.team254.frc2017.auto.modes.RamHopperShootModeBlue;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.loops.RobotStateEstimator;
import com.team254.frc2017.subsystems.*;
import com.team254.lib.util.*;
import com.team254.lib.util.math.RigidTransform2d;

import java.util.Arrays;

/**
 * The main robot class, which instantiates all robot parts and helper classes and initializes all loops. Some classes
 * are already instantiated upon robot startup; for those classes, the robot gets the instance as opposed to creating a
 * new object
 * 
 * After initializing all robot parts, the code sets up the autonomous and teleoperated cycles and also code that runs
 * periodically inside both routines.
 * 
 * This is the nexus/converging point of the robot code and the best place to start exploring.
 * 
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
    // Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    
    private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;

    // Create subsystem manager
    private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance()));

    // Initialize other helper objects

    private Looper mEnabledLooper = new Looper();

    public double findScrub(double robotDiameter, double encodercountsPer10Rots, double countsToFeet){
    	return encodercountsPer10Rots / countsToFeet / robotDiameter * 10 * Math.PI;
    }
    public Robot() {
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
        mDrive.zeroSensors();
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            
            mEnabledLooper.register(RobotStateEstimator.getInstance());


            // Pre calculate the paths we use for auto.

        } catch (Throwable t) {
            throw t;
        }
        zeroAllSensors();
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     * 
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit() {
        try {
            
            zeroAllSensors();

            mAutoModeExecuter = null;
            
            mDrive.setBrakeMode(true);

            mEnabledLooper.start();
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(new RamHopperShootModeBlue());
            mAutoModeExecuter.start();

        } catch (Throwable t) {
            throw t;
        }
    }
    public static Joystick L = new Joystick(2);
    public static Joystick R = new Joystick(1);
    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
    	//mDrive.setOpenLoop(L.getRawAxis(1), R.getRawAxis(1));
        allPeriodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            // Start loopers
            mEnabledLooper.start();
            mDrive.setBrakeMode(false);
            // Shift to high
            zeroAllSensors();
        } catch (Throwable t) {
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     * 
     * The code uses state machines to ensure that no matter what buttons the driver presses, the robot behaves in a
     * safe and consistent manner.
     * 
     * Based on driver input, the code sets a desired state for each subsystem. Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
    	
    }

    @Override
    public void disabledInit() {
        try {
            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

          
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        allPeriodic();
    }

    @Override
    public void testInit() {
        Timer.delay(0.5);

        
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        mRobotState.outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
        
    }
}
