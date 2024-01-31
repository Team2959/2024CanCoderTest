// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
  private static final double kSteerP = 0.4;
  private static final double kSteerI = 0.00001;
  private static final double kSteerD = 0.0;
  private static final double kSteerIZone = 1.0;
  private static final double kSteerMotorRotationsPerRevolution = 12.8;

  private CANSparkMax m_steerMotor;
  private CANcoder m_steerAbsoluteEncoder;
  // look at new REV Robotics documentation, some classes have been depricated
  private SparkRelativeEncoder m_steerEncoder;
  private SparkPIDController m_steerPIDController;

  private PIDController m_absoluteAngleController = new PIDController(0.3, 0.00002, kSteerD);

  private double m_targetAngleInRadians = 0.0;
  private boolean m_controlWithAbsoluteAngle = false;

  private static final double kHalfTrackWidthMeters = 0.571 / 2.0;
  private final Translation2d kFrontLeftLocation = new Translation2d(kHalfTrackWidthMeters, kHalfTrackWidthMeters);
  private final Translation2d kFrontRightLocation = new Translation2d(kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
  private final Translation2d kBackLeftLocation = new Translation2d(-kHalfTrackWidthMeters, kHalfTrackWidthMeters);
  private final Translation2d kBackRightLocation = new Translation2d(-kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
  private SwerveDriveKinematics m_kinematics;
  private static final double kMaxSpeedMetersPerSecond = 4;

  private XboxController m_Controller = new XboxController(0);

  private double steerOffsetRRWheel = 2.154;
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_steerMotor = new CANSparkMax(11, CANSparkMax.MotorType.kBrushless);
    m_steerMotor.restoreFactoryDefaults();
    m_steerMotor.setIdleMode(IdleMode.kCoast);

    m_steerAbsoluteEncoder = new CANcoder(1);

    m_steerEncoder = (SparkRelativeEncoder) m_steerMotor.getEncoder();

    m_steerPIDController = m_steerMotor.getPIDController();
    m_steerPIDController.setFeedbackDevice(m_steerEncoder);

    m_steerPIDController.setP(kSteerP);
    m_steerPIDController.setI(kSteerI);
    m_steerPIDController.setD(kSteerD);
    m_steerPIDController.setIZone(kSteerIZone);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_steerPIDController.setPositionPIDWrappingEnabled(true);
    m_steerPIDController.setPositionPIDWrappingMinInput(0);
    m_steerPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);

    m_kinematics = new SwerveDriveKinematics(kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation,
                kBackRightLocation);
            
	// Related MAX Swerve for steer offset adjustment and optimization
	  // public SwerveModulePosition getPosition() {
		// // Apply chassis angular offset to the encoder position to get the position
		// // relative to the chassis.
		// return new SwerveModulePosition(
			// m_drivingEncoder.getPosition(),
			// new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	  // }

	// {  For sending command to turn to steer position
    // correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // // Optimize the reference state to avoid spinning further than 90 degrees.
    // SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        // new Rotation2d(m_turningEncoder.getPosition()));

    // // Command driving and turning SPARKS MAX towards their respective setpoints.
    // m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
	// }

    enableLiveWindowInTest(false);

    SmartDashboard.putNumber("Steer Relative Encoder", 0);
    SmartDashboard.putNumber("Steer Absolute Encoder", 0);

    SmartDashboard.putBoolean("Reset Relative Encoder", false);

    SmartDashboard.putNumber("Target Angle", 0);

    SmartDashboard.putBoolean("AE True/RE False", false);

    setRelativeEncoderConversion();
    computeCurrentRelativeEncoderAngle();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Steer Relative Encoder", Rotation2d.fromRadians(m_steerEncoder.getPosition()).getDegrees());
    SmartDashboard.putNumber("Steer Absolute Encoder Raw", m_steerAbsoluteEncoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("Steer Absolute Encoder", getAbosultePositionInDegrees());
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
    m_steerMotor.set(0);

    double targetAngleInDegress = SmartDashboard.getNumber("Target Angle", 0);
    m_targetAngleInRadians = Rotation2d.fromDegrees(targetAngleInDegress).getRadians();
    setRelativeEncoderConversion();
    m_controlWithAbsoluteAngle = SmartDashboard.getBoolean("AE True/RE False",false);
    m_absoluteAngleController.setSetpoint(m_targetAngleInRadians);
  }

  private void setRelativeEncoderConversion() {
    m_steerEncoder.setPositionConversionFactor(2 * Math.PI / kSteerMotorRotationsPerRevolution);
  }

  private double getAbosultePositionInDegrees()
  {
    double rotations = m_steerAbsoluteEncoder.getAbsolutePosition().getValue();
    double degrees = rotations * 360;
    if (degrees < 0)
      degrees += 360;

    return degrees;
  }

  private void computeCurrentRelativeEncoderAngle()
  {
    double absoluteInRadians = Rotation2d.fromDegrees(getAbosultePositionInDegrees()).getRadians();

    double startingAngle = steerOffsetRRWheel - absoluteInRadians;

    if (startingAngle < 0)
    {
      startingAngle = startingAngle + (2 * Math.PI);
    }

    startingAngle = 2 * Math.PI - startingAngle;

    m_steerEncoder.setPosition(startingAngle);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double absoluteInDegrees = getAbosultePositionInDegrees();
    double absoluteInRadians = Rotation2d.fromDegrees(absoluteInDegrees).getRadians();

    if (m_controlWithAbsoluteAngle)
    {
      double raw = m_absoluteAngleController.calculate(absoluteInRadians);
      m_steerMotor.set(raw);
    }
    else
    {
      SwerveModuleState desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(m_targetAngleInRadians));
      SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_steerEncoder.getPosition()));

      // Command driving and turning SPARKS MAX towards their respective setpoints.
      m_steerPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
      // m_steerPIDController.setReference(m_targetAngleInRadians, CANSparkMax.ControlType.kPosition);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_steerMotor.set(0);
    setRelativeEncoderConversion();

    computeCurrentRelativeEncoderAngle();

    m_steerPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // We getY() here because of the FRC coordinate system being turned 90 degrees
    var xMetersPerSecond = -m_Controller.getLeftY() * kMaxSpeedMetersPerSecond;
    // We getX() here becasuse of the FRC coordinate system being turned 90 
    var yMetersPerSecond = -m_Controller.getLeftX() * kMaxSpeedMetersPerSecond;

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
      new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, 0));
    // SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelative
    //             ? ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond,
    //                     getAngle())
    //             : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    var frontLeftState = states[0];
    // m_steerPIDController.setReference(frontLeftState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    SwerveModuleState state = SwerveModuleState.optimize(frontLeftState,
      new Rotation2d(m_steerEncoder.getPosition()));

    m_steerPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_steerMotor.set(0);
    m_steerEncoder.setPositionConversionFactor(1.0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (SmartDashboard.getBoolean("Reset Relative Encoder", false))
    {
      m_steerEncoder.setPosition(0);
      m_steerMotor.set(0);
      SmartDashboard.putBoolean("Reset Relative Encoder", false);
      return;
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
