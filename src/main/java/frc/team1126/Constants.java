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
import frc.lib.Toolbox.AprilTag;

import java.util.HashMap;
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

    public static class AprilTags { //CHANGE THESE FOR THIS YEAR"S GAME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        public static final int SOURCE_RIGHT_BLUE_ID = 1;
        public static final int SOURCE_LEFT_BLUE_ID = 2;
        public static final int SOURCE_RIGHT_RED_ID = 9;
        public static final int SOURCE_LEFT_RED_ID = 10;
        public static final int SPEAKER_1_RED_ID = 4;
        public static final int SPEAKER_1_BLUE_ID = 7;
        public static final int SPEAKER_2_RED_ID = 3;
        public static final int SPEAKER_2_BLUE_ID = 8;
        public static final int AMP_RED_ID = 5;
        public static final int AMP_BLUE_ID = 6;
        public static final int STAGE_1_RED_ID = 12;
        public static final int STAGE_2_RED_ID = 11;
        public static final int STAGE_3_RED_ID = 13;
        public static final int STAGE_1_BLUE_ID = 15;
        public static final int STAGE_2_BLUE_ID = 14;
        public static final int STAGE_3_BLUE_ID = 16;
        public AprilTag[] tags = new AprilTag[16];
        public static final HashMap<Integer, AprilTag> TAG_MAP = new HashMap<Integer, AprilTag>() {
            {
                put(SOURCE_RIGHT_BLUE_ID, new AprilTag() {
                    {
                        id = SOURCE_RIGHT_BLUE_ID;
                        name = "Source Right Blue";
                        position = new Pose2d(new Translation2d(19.375, 48.125), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(SOURCE_LEFT_BLUE_ID, new AprilTag() {
                    {
                        id = SOURCE_LEFT_BLUE_ID;
                        name = "Source Left Blue";
                        position = new Pose2d(new Translation2d(-19.375, 48.125), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(SOURCE_RIGHT_RED_ID, new AprilTag() {
                    {
                        id = SOURCE_RIGHT_RED_ID;
                        name = "Source Right Red";
                        position = new Pose2d(new Translation2d(19.375, 48.125), Rotation2d.fromRadians(0.0));
                    }
                });
                put(SOURCE_LEFT_RED_ID, new AprilTag() {
                    {
                        id = SOURCE_LEFT_RED_ID;
                        name = "Source Left Red";
                        position = new Pose2d(new Translation2d(-19.375, 48.125), Rotation2d.fromRadians(0.0));
                    }
                });
                put(SPEAKER_1_RED_ID, new AprilTag() {
                    {
                        id = SPEAKER_1_RED_ID;
                        name = "Speaker 1 Red";
                        position = new Pose2d(new Translation2d(0,56.875), Rotation2d.fromRadians(0.0));
                    }
                });
                put(SPEAKER_1_BLUE_ID, new AprilTag() {
                    {
                        id = SPEAKER_1_BLUE_ID;
                        name = "Speaker 1 Blue";
                        position = new Pose2d(new Translation2d(0,56.875), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(SPEAKER_2_RED_ID, new AprilTag() {
                    {
                        id = SPEAKER_2_RED_ID;
                        name = "Speaker 2 Red";
                        position = new Pose2d(new Translation2d(17, 56.875), Rotation2d.fromRadians(0.0));
                    }
                });
                put(SPEAKER_2_BLUE_ID, new AprilTag() {
                    {
                        id = SPEAKER_2_BLUE_ID;
                        name = "Speaker 2 Blue";
                        position = new Pose2d(new Translation2d(17, 56.875), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(AMP_RED_ID, new AprilTag() {
                    {
                        id = AMP_RED_ID;
                        name = "Amp Red";
                        position = new Pose2d(new Translation2d(0, 48.125), Rotation2d.fromRadians(0.0));
                    }
                });
                put(AMP_BLUE_ID, new AprilTag() {
                    {
                        id = AMP_BLUE_ID;
                        name = "Amp Blue";
                        position = new Pose2d(new Translation2d(0, 48.125), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(STAGE_1_RED_ID, new AprilTag() {
                    {
                        id = STAGE_1_RED_ID;
                        name = "Stage 1 Red";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromRadians(0.0));
                    }
                });
                put(STAGE_2_RED_ID, new AprilTag() {
                    {
                        id = STAGE_2_RED_ID;
                        name = "Stage 2 Red";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromRadians(0.0));
                    }
                });
                put(STAGE_3_RED_ID, new AprilTag() {
                    {
                        id = STAGE_3_RED_ID;
                        name = "Stage 3 Red";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromRadians(0.0));
                    }
                });
                put(STAGE_1_BLUE_ID, new AprilTag() {
                    {
                        id = STAGE_1_BLUE_ID;
                        name = "Stage 1 Blue";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(STAGE_2_BLUE_ID, new AprilTag() {
                    {
                        id = STAGE_2_BLUE_ID;
                        name = "Stage 2 Blue";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromDegrees(0.0));
                    }
                });
                put(STAGE_3_BLUE_ID, new AprilTag() {
                    {
                        id = STAGE_3_BLUE_ID;
                        name = "Stage 3 Blue";
                        position = new Pose2d(new Translation2d(0, 47.5), Rotation2d.fromDegrees(0.0));
                    }
                });
            }
        };
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

    }

    public static final class ArmConstants {
        public static final int TURN_ONE_ID = 31;
        public static final int TURN_TWO_ID = 32;
        public static final int ELEVATOR_ID = 33;
    }

    public static final class ClimbConstants {
        public static final int CLIMB_ID = 41;
    }

    public static final class CoralConstants {
        public static final int CORAL_WHEELS_ID = 51;
        public static final int CORAL_PIVOT_ID = 52;
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
  
