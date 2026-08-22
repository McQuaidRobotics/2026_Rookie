package igknighters.constants;

import com.ctre.phoenix6.CANBus;

public class SubsystemConstants {

    public static final int firstMotorID = 15;
    public static final boolean disableAllLogs = false;
    public static final CANBus superStructure = new CANBus("SuperStructureBus");
    public static final CANBus drive = new CANBus("DriveBus");

    public static class kClimber {
        public static final CANBus CANBUS = SubsystemConstants.superStructure;

        public static class kChainsaw {
            public static final int LEFT_MOTOR_ID = 15;
            public static final int BUMPER_SENSOR_ID = 5;
            public static final int MAX_HEIGHT_SENSOR_ID = 8;
            public static final int MIN_HEIGHT_SENSOR_ID = 6;
            public static final int MIDDLE_HEIGHT_SENSOR_ID = 7;
            public static boolean disableChainsawLogs = true;

            static {
                if (disableAllLogs) {
                    disableChainsawLogs = true;
                }
            }

            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.05; // this is made up
            public static final double kP = 0.8;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.2;
            public static final double kG = 0.2;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double MAX_JERK = 0.05;
            public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.50;
            public static final boolean inverted = false;
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.20;
            public static final double GEAR_RATIO = 25.0;
            public static final double INCHES_TO_ROTATIONS =
                    1 / 4.5; // 1 rot at end of gearbox is 4.5 inches of linear movement
            public static final double ROTATIONS_TO_INCHES = 4.5;
            public static final double MAX_HEIGHT_INCHES = 20.0;
            public static final double MIN_HEIGHT_INCHES = 0.0;
            public static final double MIDDLE_HEIGHT_INCHES = 10.0;
            public static final double LENGTH_METERS = 0.5;
            public static final double MASS = 4.0;
            public static final double PEAK_FORWARD_CURRENT_LIMIT = 40;
            public static final double PEAK_REVERSE_CURRENT_LIMIT = 30;
            public static final double STATOR_CURRENT_LIMIT = 50;
            public static final double SUPPLY_CURRENT_LIMIT = 50;
        }

        public static class kServos {
            public static final int SERVO_PORT_1 = 9;
            public static final double MAX_ANGLE_DEGREES = 90.0;
            public static final double MIN_ANGLE_DEGREES = 0.0;
        }
    }

    public static class kIndexer {
        public static final CANBus CANBUS = SubsystemConstants.superStructure;

        public static class kSpindexer {
            public static final int LEADER_MOTOR_ID = 16;
            public static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inch radius
            public static final double GEAR_RATIO = 4.0;
            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.02;
            public static final double MAX_SPEED_RPM = 5000.0;
            public static final double MAX_ACCELERATION_RPM = 1500.0;
            public static final double MOTION_MAGIC_JERK = 600.0;
            public static final int BEAM_BREAK_SENSOR_CHANNEL = 0;
            public static final int FORWARD_CURRENT_LIMIT = 40;
            public static final int REVERSE_CURRENT_LIMIT = 30;
            public static final int STATOR_CURRENT_LIMIT = 35;
            public static final int SUPPLY_CURRENT_LIMIT = 25;
            public static final int PEAK_CURRENT_LIMIT = 40;
            public static final double kP = 0.5;
            public static final double kI = 0;
            public static final double kD = 0.001;
            public static final double kS = 0.178;
            public static final double kV = 0.5;
            public static final double kA = 0;
            public static boolean disableSpindexerLogs = true;

            static {
                if (disableAllLogs) {
                    disableSpindexerLogs = true;
                }
            }
        }

        public static class kExitRollers {
            public static final int LEADER_MOTOR_ID = 17;
            public static final double GEAR_RATIO = 5.0;
            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.02;
            public static final double MAX_SPEED_RPM = 5000.0;
            public static final double MAX_ACCELERATION_RPM = 1500.0;
            public static final double MOTION_MAGIC_JERK = 5000.0;
            public static final int BEAM_BREAK_SENSOR_CHANNEL = 0;
            public static final double kP = 0.3; // .5 max
            public static final double kI = 0.1;
            public static final double kD = 0.0;
            public static final double kS = 0.3;
            public static final double kV = 0.1;
            public static final double kA = 0.02;
            public static final double PEAK_CURRENT_LIMIT = 40;
            public static final double SUPPLY_CURRENT_LIMIT = 30;
            public static boolean disableExitRollersLogs = true;

            static {
                if (disableAllLogs) {
                    disableExitRollersLogs = true;
                }
            }
        }
    }

    public static class kIntake {
        public static final CANBus CANBUS = SubsystemConstants.superStructure;

        public static class kRollers {
            // will be configured such that + voltage will intake game pieces
            public static final int LEADER_MOTOR_ID = 20;
            public static final int FOLLOWER_MOTOR_ID = 28;
            public static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inch radius
            public static final double GEAR_RATIO = 1.0;
            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.02;
            public static final double MAX_SPEED_RPM = 5000.0;
            public static final double MAX_ACCELERATION_RPM = 1500.0;
            public static final double MOTION_MAGIC_JERK = 500.0;
            public static final int BEAM_BREAK_SENSOR_CHANNEL = 0;
            public static final int FORWARD_CURRENT_LIMIT = 40;
            public static final int REVERSE_CURRENT_LIMIT = 30;
            public static final int STATOR_CURRENT_LIMIT = 35;
            public static final int SUPPLY_CURRENT_LIMIT = 25;
            public static final double kP = 0.3; // .5 max
            public static final double kI = 0.1;
            public static final double kD = 0.0;
            public static final double kS = 0.6;
            public static final double kV = 0.15;
            public static final double kA = 0.02;
            public static boolean disableRollersLogs = true;

            static {
                if (disableAllLogs) {
                    disableRollersLogs = true;
                }
            }
        }

        public static class kPivot {
            public static final int MOTOR_ID = 18;
            public static final int CANCODER_ID = 21;
            public static final double GEAR_RATIO = 15.0;
            public static final double MAX_ANGLE_DEGREES = 66.0;
            public static final double MIN_ANGLE_DEGREES = 0.0;
            public static final double MAX_SPEED_METERS_PER_SECOND = 1.0;
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = .5;
            public static final double ENCODER_OFFSET = 0.31982421875;
            public static final double MAX_JERK = 1.0;
            public static final int STATOR_CURRENT_LIMIT = 20;
            public static final int SUPPLY_CURRENT_LIMIT = 20;
            public static final int SUPPLY_UPPER_LIMIT = 25;
            public static final double kP = 45.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double JKG_M2 = 0.01;
            public static final double LENGTH_METERS =
                    .25; // distance from central shaft to edge of wrist
            public static boolean disablePivotLogs = false;

            static {
                if (disableAllLogs) {
                    disablePivotLogs = true;
                }
            }
        }
    }

    public static class kShooter {
        public static final CANBus CANBUS = SubsystemConstants.superStructure;

        public static class kFlywheels {
            public static final int LEADER_MOTOR_ID = 23;
            public static final int FOLLOWER_MOTOR_ID = 22;
            public static final double WHEEL_RADIUS_METERS = 0.05; // 5 cm
            public static final double GEAR_RATIO = 1.0;
            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.02;
            public static final double MAX_SPEED_RPM = 5000.0;
            public static final double MAX_ACCELERATION_RPM = 3000.0;
            public static final double MOTION_MAGIC_JERK = 100.0;
            public static final int BEAM_BREAK_SENSOR_CHANNEL = 0;
            public static final double kP = 0.3; // .5 max
            public static final double kI = 0.1;
            public static final double kD = 0.0;
            public static final double kS = 0.17;
            public static final double kV = 0.1;
            public static final double kA = 0.02;
            public static final double ShooterHeightMeters =
                    .5; // 40 cm this is made up it will be off ground though
            public static final double PEAK_CURRENT_LIMIT = 40;
            public static final double SUPPLY_CURRENT_LIMIT = 30;
            public static final double RPM_TO_METERS_PER_SECOND_FACTOR = 0;
            public static boolean disableFlywheelsLogs = false;

            // RPM_TO_METERS_PER_SECOND_FACTOR removed; use
            // Robot.consts.shooter().kFlywheels().RPM_TO_METERS_PER_SECOND_FACTOR()

            static {
                if (disableAllLogs) {
                    disableFlywheelsLogs = true;
                }
            }
        }

        public static class kTurret {
            public static final int MOTOR_ID = 24;
            public static final int CANCODER_ID = 25;
            public static final double CANCODER_OFFSET_ROTATIONS = 0.742919921875;

            public static final double xMeterOffset = 7 * Conv.INCHES_TO_METERS;
            public static final double yMeterOffset = 7 * Conv.INCHES_TO_METERS;
            public static final double GEAR_RATIO = 16.2;
            public static final double MAX_ANGLE_DEGREES = 90.0;
            public static final double MIN_ANGLE_DEGREES = -270.0;
            public static final double MAX_SPEED_RPM = 600.0;
            public static final double MAX_ACCELERATION_RPM = 800.0;
            public static final double MAX_JERK = 300;
            public static final int STATOR_CURRENT_LIMIT = 40;
            public static final int SUPPLY_CURRENT_LIMIT = 30;
            public static final double kP = 55.0; // tuned
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static boolean disableTurretLogs = false;

            static {
                if (disableAllLogs) {
                    disableTurretLogs = true;
                }
            }
        }

        public static class kHood {
            public static final int MOTOR_ID = 26;
            public static final double MOTOR_ROTS_TO_HOOD_DEGREES = 15.0;
            public static final double MAX_ANGLE_DEGREES = 52.855225;
            public static final double MIN_ANGLE_DEGREES = 18.6;
            public static final double MAX_SPEED_R_P_S = 12.0;
            public static final double MAX_ACCEL_R_P_S_S = 24.0;
            public static final double MAX_JERK = 1.0;
            public static final int STATOR_CURRENT_LIMIT = 30;
            public static final int SUPPLY_CURRENT_LIMIT = 20;
            public static final double kP = 4.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.27;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double JKG_M2 = 0.01;
            public static final double LENGTH_METERS =
                    .25; // distance from central shaft to edge of flap
            public static final int REVERSE_LIMIT_SWITCH_ID = 9;
            public static boolean disableHoodLogs = true;

            static {
                if (disableAllLogs) {
                    disableHoodLogs = true;
                }
            }
        }
    }

    public static class kLimelightVision {
        public static final String turretCam = "limelight";
        public static final String intakeCam = "limelight-intake";
        public static final String backCam = "limelight-back";
        public static final String rightCam = "limelight-left";
        public static boolean disableVisionLogs = true;

        static {
            if (disableAllLogs) {
                disableVisionLogs = true;
            }
        }
    }
}
