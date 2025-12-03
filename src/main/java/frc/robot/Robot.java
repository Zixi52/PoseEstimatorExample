// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Mini-autonomous target (meters and degrees)
  private final double targetX = 3.0;
  private final double targetY = 3.5;
  private final double targetAngle = 0.0;

  private final double obstacleX = 1.5;
  private final double obstacleY = 1.75;
  private final double obstacleRadius = 0.2;

  private final double kAvoid = 0.5;

  /** Converts current pose into x/y/rotation speeds toward target. */
  private ChassisSpeeds calculateAutoSpeeds(Pose2d pose) {
    // Simple proportional gains
    double kP_pos = 1.2;
    double kP_rot = 0.04;

    double errorX = targetX - pose.getX();
    double errorY = targetY - pose.getY();
    double errorTheta = targetAngle - pose.getRotation().getDegrees();

    double xSpeed = kP_pos * errorX;
    double ySpeed = kP_pos * errorY;
    double rotSpeed = kP_rot * errorTheta;

    // clamp to drivetrain max speeds
    xSpeed = Math.max(-Drivetrain.kMaxSpeed, Math.min(Drivetrain.kMaxSpeed, xSpeed));
    ySpeed = Math.max(-Drivetrain.kMaxSpeed, Math.min(Drivetrain.kMaxSpeed, ySpeed));
    rotSpeed = Math.max(-Drivetrain.kMaxAngularSpeed, Math.min(Drivetrain.kMaxAngularSpeed, rotSpeed));

    return new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
  }


  @Override
  public void autonomousPeriodic() {
    // 1. Update odometry (includes vision)

    // 2. Get the current estimated pose and store in a Pose2d variable called currentPose

    // 3. Calculate speeds toward the target and store in a ChassisSpeeds variable called speeds

    // 4. Calculate distance to obstacle, already written for you
    double distanceToObstacle = currentPose.getTranslation().getDistance(
      new Translation2d(obstacleX, obstacleY)
    );

    // 5. Calculate the additional speeds necessary to avoid obstacle. Create an avoidY variable here.
    double avoidX = 0;
    if (distanceToObstacle < obstacleRadius) {
      // Push away from obstacle
      avoidX = kAvoid * (currentPose.getX() - obstacleX);
    }

    // 6. Drive the robot toward the target (field-relative). What is missing here?
    m_swerve.drive(
        speeds.vxMetersPerSecond + avoidX,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        true,
        getPeriod()
    );
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(m_controller.getLeftX()) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
