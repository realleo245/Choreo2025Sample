
package frc.robot.subsystems;


import choreo.trajectory.SwerveSample;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;


/**
 * This isn't how you write a swerve drive class!!!!!!!!!!!!!!!!!!!!!!
 * @param forward
 * @param strafe
 * @param rotation
 * @param fieldRelative
 * @param slowMode
 * @param isOpenLoop
 */
public class SwerveDrive extends SubsystemBase {

   

    public SwerveDrive() {
       
    }

    public void drive(double forward, double strafe, double rotation, boolean fieldRelative, boolean slowMode,
            boolean isOpenLoop) {
        
    }

    /**
     * Used for auton
     */
    public void setChassisSpeeds(ChassisSpeeds desiredSpeeds) {
        
    }

   
    /**
     * Used by PathPlanner to get current robot speed
     * @return
     */
    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds();
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[1];
        
        return states;
    }

    /**
     * Resets the pose and also field relative gyro offset
     */
    public void resetPoseAndGyroOffset(Pose2d newPose) {
        Rotation2d rotation = newPose.getRotation();
        
        zeroHeading();
        resetPose(newPose);
    }

    /**
     * Resets the pose to the given pose
     */
    public void resetPose(Pose2d newPose) {
        
    }

    public Pose2d getPose() {
        return new Pose2d();
    }

    public Pose2d getPose(double timestamp) {
        return new Pose2d();
    }


    /**
     * Zeroes the heading of the robot.
     * This method also resets the pose so that it doesn't appear
     * that the robot has changed positon.
     */
    public void zeroHeading() {
    
        resetPose(getPose());
    }

    public Rotation2d getFieldRelativeAngle() {
        return new Rotation2d();
    }
    // Mostly copied from the release notes
    public void choreoController(Pose2d curPose, SwerveSample sample) {
        PIDController xController = new PIDController(0.2, 0, 0);
        PIDController yController = new PIDController(0.2, 0, 0);
        PIDController rController = new PIDController(0.2, 0, 0);
        
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(curPose.getX(), sample.x) + sample.vx,
                yController.calculate(curPose.getY(), sample.y) + sample.vy,
                rController.calculate(curPose.getRotation().getRadians(), sample.heading) + sample.omega
            ), curPose.getRotation());
      this.setChassisSpeeds(speeds);
    }
    // public ChassisSpeeds choreoController(Pose2d curPose, SwerveSample sample) {
    //     PIDController xController = new PIDController(0.2, 0, 0);
    //     PIDController yController = new PIDController(0.2, 0, 0);
    //     PIDController rController = new PIDController(0.2, 0, 0);
    //     ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //         new ChassisSpeeds(
    //             xController.calculate(curPose.getX(), sample.x) + sample.vx,
    //             yController.calculate(curPose.getY(), sample.y) + sample.vy,
    //             rController.calculate(curPose.getRotation().getRadians(), sample.heading) + sample.omega
    //         ), curPose.getRotation());
    //   this.setChassisSpeeds(speeds);
    //     return speeds;
    // }
    
    @Override
    public void periodic() {
        
    }
}

