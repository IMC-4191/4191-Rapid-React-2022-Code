// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(9);
  // WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(8);
  WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(6);
  WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(7);
  MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  // WPI_VictorSPX m_frontRight = new WPI_VictorSPX(7);
  // WPI_VictorSPX m_rearRight = new WPI_VictorSPX(6);
  WPI_VictorSPX m_frontRight = new WPI_VictorSPX(8);
  WPI_VictorSPX m_rearRight = new WPI_VictorSPX(9);
  MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_frontRight, m_rearRight);

  DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  
  Joystick spaceX = new Joystick(2);  // intake, hopper and climb controller
  Joystick driveController = new Joystick(1); // drivetrain and shooter controller

  WPI_TalonSRX intake = new WPI_TalonSRX(5);
  // WPI_VictorSPX intakeLauncher = new WPI_VictorSPX(9);
  WPI_VictorSPX hopper = new WPI_VictorSPX(10);

  WPI_TalonSRX shooter1 = new WPI_TalonSRX(3);
  WPI_TalonSRX shooter2 = new WPI_TalonSRX(4);
  MotorControllerGroup shooter = new MotorControllerGroup(shooter1, shooter2);

  WPI_TalonSRX climb1 = new WPI_TalonSRX(2);
  WPI_TalonSRX climb2 = new WPI_TalonSRX(1);
  MotorControllerGroup climb = new MotorControllerGroup(climb1, climb2);
  Spark climb_up = new Spark(9);

  Timer myAwesomeTimer = new Timer();
  boolean hasValidTarget = false;
  double steer_p = 0.0f; // try
  double steer_d = 0.0f; // try
  double steer_output = 0.0f;
  double steer_error = 0.0f;
  double steer_last_error = 0.0f;

  @Override
  public void robotInit() {
    // CameraServer.getInstance().startAutomaticCapture();
    m_rightDrive.getInverted();
    shooter1.getInverted();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    myAwesomeTimer.reset();
    myAwesomeTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if (myAwesomeTimer.get() > 0.2 && myAwesomeTimer.get() < 0.5) {
    //   intakeLauncher.set(-1);
    // }

    if (myAwesomeTimer.get() < 0.5) {
      m_drive.arcadeDrive(0.0, -0.5);   // go back
    }
    else if (myAwesomeTimer.get() < 2) {
      m_drive.arcadeDrive(0, 0);        // wait
    }
    else if (myAwesomeTimer.get() < 6) {
      hopper.set(-0.45);                // turn on hopper
      shooter.set(1);                   // shoot the ball
    } else if (myAwesomeTimer.get() < 8.5) {
      m_drive.arcadeDrive(0.0, -0.55);  // taxi
    } else {
      shooter.set(0);
      hopper.set(0);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    myAwesomeTimer.stop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    // drive
    if (driveController.getRawButton(3)) {
      m_drive.arcadeDrive(0, Aim_Snake_Hack(tv, tx));
    }
    else if (driveController.getRawButton(1)) {
      m_drive.arcadeDrive(driveController.getRawAxis(2) * (0.55), driveController.getRawAxis(1) * (-0.45));
      SmartDashboard.putBoolean("Slow down more", driveController.getRawButton(1));
    }
    else if (driveController.getRawButton(6)) {
      m_drive.arcadeDrive(driveController.getRawAxis(2) * (0.65), driveController.getRawAxis(1) * (-0.68));
      SmartDashboard.putBoolean("Slow down", driveController.getRawButton(6));
    } 
    else {                        // axis 2 = rotate, 1 = go
      m_drive.arcadeDrive(driveController.getRawAxis(2) * (0.74), driveController.getRawAxis(1) * (-0.80));
    }

    // intake
    if (spaceX.getRawButton(7)) {
      intake.set(0.7);
    }
    else if (spaceX.getRawButton(8)) {
      intake.set(-0.5);
    } else {
      intake.set(0);
    }

    // hopper
    if (spaceX.getRawButton(6)) {
      hopper.set(-0.3);
    }
    else if (spaceX.getRawButton(5)) {
      hopper.set(0.3);
    }
    else if (spaceX.getRawButton(4)) {
      hopper.set(-0.6);
    } else {
      hopper.set(0);
    }

    // shooter
    if (driveController.getRawButton(4)) {
      shooter.set(0.9);
    } else {
      shooter.set(0);
    }

    // climb
    if (spaceX.getRawButton(3)) {
      climb_up.set(-0.5);
    }
    else if (spaceX.getRawButton(2)) {
      climb.set(-1);
    } else {
      climb.set(0);
      climb_up.set(0);
    }
  }

  public double Aim_Snake_Hack(double tv, double tx) {
    if (tv < 1.0) {
      hasValidTarget = false;
      steer_output = 0.0f;
    } else {
      hasValidTarget = true;
      steer_last_error = steer_error;
      steer_error = tx;
    }

    if (hasValidTarget) {
      if (steer_error > 1.0f) {
        steer_output = steer_error * steer_p + steer_last_error * steer_d;
      } else {
        steer_output = 0.0f;
      }
    }

    if (steer_output > 0.7f) {
      steer_output = 0.7f;
    }

    return steer_output;
  }


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
