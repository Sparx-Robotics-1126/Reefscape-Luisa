package frc.team1126;

// import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

import com.pathplanner.lib.config.PIDConstants;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

//import frc.lib.swervelib.math.Matter;
//import frc.lib.swervelib.parser.PIDFConfig;

public final class Constants {
    public static final boolean DEFAULT_CONSTANT_PROPERTIES = RobotBase.isReal();
    public static final double NOMINAL_VOLTAGE = 12;
    public static final int MAX_CURRENT = 80;
    public static final double DEAD_BAND = .5;

    
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final double ROBOT_WIDTH_W_BUMBERS = .762;
  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

 public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(2, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(1.5, 1, 1.15);
    public static final double MAX_MODULE_SPEED = 5.5;
  }


    public static final class Auton
    {
        public static final PIDFConfig X_AUTO_PID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig Y_AUTO_PID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig ANGLE_AUTO_PID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_SPEED        = 2;
        public static final double MAX_ACCELERATION = 2;
    }

    /* Field related constants */
    public static final class FieldConstants {
        // List of possible scoring locations as Pose2d objects
        public static final List<Pose2d> SCORING_POSITIONS =
                List.of(
                        new Pose2d(
                                new Translation2d(0.555, 7.436),
                                Rotation2d.fromRadians(Math.PI)), // Red loading double station
                        new Pose2d(new Translation2d(0.555, 6.146), Rotation2d.fromRadians(Math.PI)),
                        new Pose2d(
                                new Translation2d(15.03, 5.061),
                                Rotation2d.fromDegrees(0.0)), // Red node scoring locations
                        new Pose2d(new Translation2d(15.03, 4.405), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 3.846), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 3.298), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 2.74), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 2.2), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 1.62), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 1.06), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(new Translation2d(15.03, 0.52), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(
                                new Translation2d(15.64, 7.430),
                                Rotation2d.fromDegrees(0.0)), // Blue loading double substation
                        new Pose2d(new Translation2d(15.64, 6.16), Rotation2d.fromDegrees(0.0)),
                        new Pose2d(
                                new Translation2d(1.598, 4.996),
                                Rotation2d.fromRadians(-Math.PI)), // Blue node scoring locations
                        new Pose2d(new Translation2d(1.598, 4.373), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 3.85), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 3.3), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 2.75), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 2.2), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 1.63), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 1.05), Rotation2d.fromRadians(-Math.PI)),
                        new Pose2d(new Translation2d(1.598, 0.5), Rotation2d.fromRadians(-Math.PI)));
    }


    /**
     * General robot constants
     */
    public static final class GeneralConstants {
        // Driver controller port
        public static final int DRIVER_CONTROLLER_ID = 0;

        // Operator controller port
        public static final int OPERATOR_CONTROLLER_ID = 1;
    }

    /**
     * Constants revolving around swerve subsystem
     */
    public static class SwerveConstants {

        // Joystick axis deadband for the swerve drive

        public static final double SWERVE_DEADBAND = 0.1;

        // Swerve default translational scalar
        public static final double SWERVE_NORMAL_TRANSLATION = 0.6;

        // Swerve slow translational scalar
        public static final double SWERVE_SLOW_TRANSLATION = 0.25;

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10;

        public static final boolean IS_FIRST_ORDER = true;

        public static final double DT_CONSTANT = 0.1;

        public static final boolean HEADING_CORRECTION = false;

        public static final boolean CHASSIS_VELOCITY_CORRECTION = false;

        //new from 2025
        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
        public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
        public static final double MAX_SPEED  = Units.feetToMeters(14.5);

        //for endgame rumble feature
        public static final int ENDGAME_SECONDS = 30;
        public static final int STOP_RUMBLE_SECONDS = 28;

    }

//from team 2638 https://github.com/rebels2638/2025-Reefscape-Robot
public static final class AlignmentConstants {
    // these assume the robots volume is zero. does not take into account frame
   
    public static final double kINTER_BRANCH_DIST_METER = 0.34;
    public static final Pose2d[] kCENTER_FACES =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    static {
      // Initialize faces
      kCENTER_FACES[0] =
        //bottom
          new Pose2d(
            3.642, 
            4.024,
            Rotation2d.fromDegrees(0));
        //top right
      kCENTER_FACES[1] =
          new Pose2d(
            4.916, 
            3.285,
            Rotation2d.fromDegrees(120));
        //bottom right
      kCENTER_FACES[2] =
          new Pose2d(
            4.064,
            3.291,
            Rotation2d.fromDegrees(60));
        //top
      kCENTER_FACES[3] =
          new Pose2d(
            5.344, 
            4.023,
            Rotation2d.fromDegrees(180));
        //bottom left
      kCENTER_FACES[4] =
          new Pose2d(
            4.064, 
            4.763,
            Rotation2d.fromDegrees(-60));
        //top left
      kCENTER_FACES[5] =
          new Pose2d(
            4.912, 
            4.770,
            Rotation2d.fromDegrees(-120));
    }

    public static final Translation2d CORAL_OFFSET_FROM_ROBOT_CENTER = new Translation2d(0,0);
    
    private AlignmentConstants() {}
  }

    /**
     * Constants revolving around the vision subsystem.
     */
    public static final class VisionConstants {
        // Camera name
        public static final String CAMERA_NAME = "OV5647";
        // photonvision pipelines - change these!!!!
        public static final int FRONT_PHOTONVISION_PIPELINE = 0;
        public static final int BACK_PHOTONVISION_PIPELINE = 1;
        public static final int LEFT_PHOTONVISION_PIPELINE = 2;
        public static final int RIGHT_PHOTONVISION_PIPELINE = 3;

        public static final double VISION_FIELD_MARGIN = 0.5;
        public static final double VISION_Z_MARGIN = 0.75;
        public static final double VISION_STD_XY_SCALE = 0.02;
        public static final double VISION_STD_ROT_SCALE = 0.035;

        public static final double FIELD_LENGTH = 16.5417;
        public static final double FIELD_WIDTH = 8.0136;

        
        public static final Transform3d kFrontCameraLocation = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.507), Units.inchesToMeters(5.673),
                Units.inchesToMeters(6.789)),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(0.0)));
    
        public static final Transform3d kLeftCameraLocation = new Transform3d(
            new Translation3d(Units.inchesToMeters(-2.80), Units.inchesToMeters(12.689),
                Units.inchesToMeters(9.43)),
            new Rotation3d(180.0, Math.toRadians(-20.0), Math.toRadians(90.0)));
    
        public static final Transform3d kRightCameraLocation = new Transform3d(
            new Translation3d(Units.inchesToMeters(-2.80), Units.inchesToMeters(-12.689),
                Units.inchesToMeters(9.43)),
            new Rotation3d(180.0, Math.toRadians(-20.0), Math.toRadians(-90.0)));



        // Robot to camera transform
        public static final Transform3d ROBOT_TO_CAM =
                new Transform3d(
                        new Translation3d(0.0, Units.inchesToMeters(1.5), Units.inchesToMeters(39.0)),
                        new Rotation3d(0.0, 0.0, 0.0));
    }

    public static final class AlgaeConstants {
        public static final int ALGAE_WHEELS_ID = 21;
        public static final int ALGAE_ROTATION_ID = 22;
        public static final int ALGAE_HOME_ID =16 ; // change these
        public static final int ALGAE_SENSOR_ID = 11; // and these
    }

    public static final class ArmConstants {
        public static final int TURN_ONE_ID = 31;
        public static final int TURN_TWO_ID = 32;
        public static final int ELEVATOR_ID = 33;
        public static final int ARM_SENSOR_ID = 1; //need to be changed
        public static final int EXTENSION_SENSOR_ID = 2; //also needs changing!!
    }

    public static final class PlacerConstants {
        public static final int PLACER_ID = 34;
        public static final int PLACER_FOLLOWER_ID = 35;
        public static final int PLACER_BOTTOM_ID = 11; // CHANGE THESE!!!!!
        public static final int PLACER_TOP_ID = 12;
    }

    public static final class ClimbConstants {
        public static final int CLIMB_ID = 41;
    }

    public static final class CoralConstants {
        public static final int CORAL_WHEELS_ID = 51;
        public static final int CORAL_PIVOT_ID = 52;
    }

    public static final class PneumaticsConstants {
        public static final int MODULE_ID = 20;
        public static final int ACQ_CHANNEL = 0; // CHANGE THESE TOO!!!!!!!!!
        public static final int CLIMB_CHANNEL = 1; // <-----------
    }
    public static final class CoralSubsystemConstants {
    public static final class ElevatorSetpoints {
        public static final int kFeederStation = 0;
        public static final int kLevel1 = 0;
        public static final int kLevel2 = 0;
        public static final int kLevel3 = 100;
        public static final int kLevel4 = 150;
      }
  
      public static final class ArmSetpoints {
        public static final double kFeederStation = 33;
        public static final double kLevel1 = 0;
        public static final double kLevel2 = 2;
        public static final double kLevel3 = 2;
        public static final double kLevel4 = 19;
      }
  
      public static final class IntakeSetpoints {
        public static final double kForward = 0.5;
        public static final double kReverse = -0.5;
      }
    }
    /**
     * Constants revolving around auton modes.
     */
    // public static final class AutonConstants {

    //     public static final double MAX_VELOCITY = 3.0;
    //     public static final double MAX_ACCELERATION = 2.0;
    //     public static final PathConstraints CONSTRAINTS =
    //             new PathConstraints(AutonConstants.MAX_VELOCITY, AutonConstants.MAX_ACCELERATION);

    //     public static final double XY_CONTROLLER_P = 4;
    //     public static final double THETA_CONTROLLER_P = 1;
    // }
}
  
