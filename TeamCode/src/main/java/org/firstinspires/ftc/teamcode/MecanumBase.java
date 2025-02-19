package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

/**
 * This is a base class for a specific physical implementation of The
 * mecanum drive. What makes it specific to the physical implementation
 * of the drive is the mapping of the motors and IMU, and the
 * that control acceleration and deceleration, and that map encoder tics to
 * inches of robot movement.
 */
public abstract class MecanumBase extends LinearOpMode {
    // Added this for the claw servos
    HardwareProfile robot = new HardwareProfile();


    private DcMotor FrontRightDrive;
    private DcMotor RearRightDrive;
    private DcMotor FrontLeftDrive;
    private DcMotor RearLeftDrive;
    private BNO055IMU imu;

    // the claw servos
    private Servo IntakeLeft;
    private Servo IntakeRight;

    // the linear slide motor
    public DcMotor Carousel;

    // the conditioned gamepad stick positions
    protected double left_x;
    protected double left_y;
    protected double right_x;
    protected double right_y;
    protected double ave_x;

    // The constants that regulate this program - adjust these to your physical
    // implementation of the drive.
    double bumper_speed = 0.5;
    double mtr_accel_min = 0.4;
    double mtr_decel_min = 0.1;
    double mtr_accel_tics = 400.0;
    double mtr_decel_tics = 1200.0;
    double mtr_accel_degs = 10.0;
    double mtr_decel_degs = 60.0;
    double tics_per_inch_forward = 84.0;
    double tics_per_inch_sideways = 83.0;

    // tracking the heading of the robot
    double heading;
    int heading_revs = 0;
    double heading_raw_last;

    double closePos = 1;

    /**
     * Call this in your OpMode before the waitForStart() method to
     * initialize the mecanum traction.
     * @param hardware_map The hardware map for the configuration of your robot.
     */
    protected void initialize_pre_start(HardwareMap hwMap) {
        FrontRightDrive = hardwareMap.dcMotor.get("frontrightdrive");
        RearRightDrive = hardwareMap.dcMotor.get("rearrightdrive");
        FrontLeftDrive = hardwareMap.dcMotor.get("frontleftdrive");
        RearLeftDrive = hardwareMap.dcMotor.get("rearleftdrive");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // You will have to determine which motors to reverse for your robot. All
        // motors should propell forward when positive power is applied.
        DcMotor.ZeroPowerBehavior at_zero = DcMotor.ZeroPowerBehavior.BRAKE;
        initialize_motor(FrontRightDrive, DcMotorSimple.Direction.REVERSE, at_zero);
        initialize_motor(RearRightDrive, DcMotorSimple.Direction.FORWARD, at_zero);
        initialize_motor(FrontLeftDrive, DcMotorSimple.Direction.FORWARD, at_zero);
        initialize_motor(RearLeftDrive, DcMotorSimple.Direction.REVERSE, at_zero);
        reset_drive_encoders();
        // It grips the cone in autonomous
        IntakeLeft = hwMap.get(Servo.class, "intakeleft");
        IntakeLeft.setDirection(Servo.Direction.FORWARD);
        IntakeRight = hwMap.get(Servo.class, "intakeright");
        IntakeRight.setDirection(Servo.Direction.REVERSE);
        IntakeLeft.setPosition(closePos);
        IntakeRight.setPosition(closePos);
        Carousel = hwMap.get(DcMotor.class, "Carousel");
        // Carousel.setDirection(DcMotor.Direction.REVERSE);
        Carousel.setTargetPosition(0);
        Carousel.setPower(0);
        Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Carousel.setPower(0);
        double liftPower = .5;
        Carousel.setTargetPosition(-100);
        Carousel.setPower(liftPower);

    }

    /**
     * Call this in your OpMode after the waitForStart() method and before
     * you have your robot start to do something. This initializes things
     * whose initialization needs to happen close to the start time of
     * robot activity. IMU gyros are an example of this due to procession
     * in gyroscopes.
     */
    protected void initialize_post_start() {
        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(imu_params);
        while (!imu.isGyroCalibrated()) {
            continue;
        }
        // initialize the heading tracking
        Orientation angles = imu.getAngularOrientation();
        heading_raw_last = angles.firstAngle;
        heading = -heading_raw_last;

    }

    // ===========================================================================
    // private support functions
    // ===========================================================================
    /**
     * Initialize a motor direction and zero behavior.
     * @param mtr (DcMotor) The motor to initialize.
     * @param dir (DcMotorSimple.Direction) The direction for positive power.
     * @param at_zero (DcMotor.ZeroPowerBehavior) The behavior with 0 power.
     */
    private void initialize_motor(DcMotor mtr, DcMotorSimple.Direction dir,
                                  DcMotor.ZeroPowerBehavior at_zero) {
        mtr.setDirection(dir);
        FrontRightDrive.setZeroPowerBehavior(at_zero);
    }

    /**
     * Condition the gamepad stick values to be -1 (left and down) to 1 (right
     * and up). If the right bumper is pressed then all conditioned values are
     * scaled by the bumper_speed scale for finer control.
     */
    protected void condition_sticks() {
        left_x = gamepad1.left_stick_x;
        left_y = -gamepad1.left_stick_y;
        right_x = gamepad1.right_stick_x;
        right_y = -gamepad1.right_stick_y;
        if (gamepad1.right_bumper) {
            right_x = right_x * bumper_speed;
            right_y = right_y * bumper_speed;
            left_x = left_x * bumper_speed;
            left_y = left_y * bumper_speed;
        }
        ave_x = (left_x + right_x) * 0.5;
    }

    /**
     * Explicitly set the power (really the speed because the motors are running
     * with encoders) for each of the motors and update the robot heading.
     * @param power_rf (double) The power (speed) for the right-front motor.
     * @param power_rr (double) The power (speed) for the right-rear motor.
     * @param power_lf (double) The power (speed) for the left-front motor.
     * @param power_lr (double) The power (speed) for the left-rear motor.
     */
    private void set_power(double power_rf, double power_rr,
                           double power_lf, double power_lr) {
        FrontRightDrive.setPower(power_rf);
        RearRightDrive.setPower(power_rr);
        FrontLeftDrive.setPower(power_lf);
        RearLeftDrive.setPower(power_lr);

        // anytime power is being changed the heading may change. The IMU goes from
        // -180.0 to 180.0. The discontinuity at 180,-180 is a programming headache.
        // if you rotate through that is takes a bunch of special programming logic
        // to figure out where you are. Instead, we will monitor going through that
        // discontinuity and increment a rotation counter so our heading will start
        // at 0 when the IMU is initialized, and be a continuous function from
        // -infinity to + infinity.
        Orientation angles = imu.getAngularOrientation();
        double heading_raw = angles.firstAngle;
        if (heading_raw_last < -140.0 && heading_raw > 0.0) {
            heading_revs -= 1;
        } else if (heading_raw_last > 140.0 && heading_raw < 0.0) {
            heading_revs += 1;
        }
        // Our mental model says clockwise rotation (turning right) is a positive
        // rotation for the power, so we will sign correct heading to match.
        heading = -(heading_revs * 360.0 + heading_raw);
        heading_raw_last = heading_raw;
        telemetry.addData("revs", heading_revs);
        telemetry.addData("heading", heading);
    }

    /**
     * Set speeds based on the abstraction of robot motion being a combination
     * of forward, sideways, and rotation in the range -1 to 1 where
     * -1 is the fastest the robot can move in the negative direction, 0 is
     * stopped for that direction.
     * @param forward (double) The forward speed in the range -1 (full speed
     *  backwards) to 1 (full speed forward).
     * @param sideways (double) The sideways speed in the range -1 (full speed
     *  left) to 1 (full speed right).
     * @param rotation (double) The rotation speed in the range -1 (full speed
     *  counter-clockwise) to 1 (full speed clockwise)
     */
    protected void set_speeds(double forward, double sideways, double rotation) {
        double max= Math.abs(forward) + Math.abs(sideways) + Math.abs(rotation);
        double scale = (max <= 1.0) ? 1.0 : 1.0 / max;
        set_power(scale * (forward - sideways - rotation),  // power_rf
                scale * (forward + sideways - rotation),  // power_rr
                scale * (forward + sideways + rotation),  // power_lf
                scale * (forward - sideways + rotation)); // power_lr
    }

    /**
     * Reset the drive encoders to 0 and set all motors to RUN_USING_ENCODER,
     * which means that when you set a motor power it is really the motor speed
     * and the motor control libraries use the encoders and PID loops to keep
     * the motors coordinated.
     */
    private void reset_drive_encoders() {
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ===========================================================================
    // driver control functions
    // ===========================================================================
    /**
     * Use the right and left stick Y positions to control the forward-backward
     * speed of the right and left wheels respectively. The average stick Y
     * position controls sideways speed.
     */
    protected void tank_drive() {
        double scale = 1.0;
        double max_left = Math.abs(left_y) + Math.abs(ave_x);
        double max_right = Math.abs(right_y) + Math.abs(ave_x);
        if (max_left > 1.0) {
            scale = (max_right > max_left) ? 1.0 / max_right : 1.0 / max_left;
        } else if (max_right > 1.0) {
            scale = 1.0 / max_right;
        }
        set_power( scale * (right_y - ave_x),   // power_rf
                scale * (right_y + ave_x),   // power_rr
                scale * (left_y + ave_x),    // power_lf
                scale * (left_y - ave_x));   // power_lr
    }

    // ===========================================================================
    // autonomous control functions
    // ===========================================================================
    /**
     * This is a ramp-in and ramp-out generator function that returns a power
     * for the current position in the move. This method assumes the start
     * position is 0, end is the target, and current is the current position.
     *
     * @param current (double) The current position in the range 0 to target.
     * @param target (double) The target final position.
     * @param mtr_accel_min (double) The minimum acceleration motor speed - to
     *  assure the robot gets to the target.
     * @param mtr_decel_min (double) The minimum deceleration motor speed - to
     *  assure the robot gets to the target.
     * @param accel (double) The acceleration distance with power at current=0
     *  starting at mtr_min and power at current=accel reaching 1.0.
     * @param decel (double) The deceleration distance with power at
     *  current=target-decel being 1.0 and power at current=target being mtr_min.
     * @return (double) Returns the power that will be in the range
     *  mtr_min to 1.0
     */
    private double power_accel_decel(double current, double target,
                                     double mtr_accel_min, double mtr_decel_min,
                                     double accel, double decel) {
        if (current <= 0.0) {
            // could happen if there was some robot motion (was hit or coasting)
            // that confused the sensor/logic.
            return mtr_accel_min;
        } else if (current >= target) {
            // could happen if there was some robot motion (was hit or coasting)
            // that confused the sensor/logic.
            return 0.0;
        }
        double mtr_tmp = 1.0;
        if (current < accel) {
            // in the acceleration zone
            mtr_tmp = mtr_accel_min + (1.0 - mtr_accel_min) * (current / accel);
        }
        if (current > target - decel) {
            // in the deceleration zone
            double mtr_tmp_2 = mtr_decel_min +
                    (1 - mtr_decel_min) * ((target - current) / decel);
            if (mtr_tmp_2 < mtr_tmp) {
                // The deceleration is less than the acceleration or the 1.0 max.
                mtr_tmp = mtr_tmp_2;
            }
        }
        return mtr_tmp;
    }

    /**
     * Get the forward tics, which is the sum of the encoders for all
     * motors. This is a start on redundancy (using all 4 encoders),
     * however, if an encoder fails this will still not detect that.
     */
    private double forward_tics() {
        return FrontRightDrive.getCurrentPosition() + FrontLeftDrive.getCurrentPosition() +
                RearRightDrive.getCurrentPosition() + RearLeftDrive.getCurrentPosition();
    }

    /**
     * Get the sideways tics which is the sum if the encoders corrected
     * for direction of rotation when moving sideways. This is a start on
     * redundancy (using all 4 encoders), however, if an encoder fails
     * this will still not detect that.
     */
    private double sideways_tics() {
        return (RearRightDrive.getCurrentPosition() + FrontLeftDrive.getCurrentPosition()) -
                (FrontRightDrive.getCurrentPosition() + RearLeftDrive.getCurrentPosition());
    }

    /**
     * Move the specified distance in the the specified direction (degrees) with no rotation.
     * @param inches (double) The distance in inches to move along the specified
     *  direction. This may be a negative distance. NOTE that moving
     *  in a negative distance is the same as changing the direction
     *  180&deg; degrees and moving in a positive direction.
     * @param degrees (double) The direction the robot should move: 0.0&deg; is straight
     *  ahead; 90.0&deg; is sideways to the right; -90.0&deg; is sideways to the
     *  left; 180&deg; (-180&deg;) is backwards; e.g. 45.0&deg; would be diagonally
     *  forward and to the right.
     */
    protected void move(double inches, double degrees) {
        reset_drive_encoders();
        double radians = Math.toRadians(degrees);
        double sin = Math.sin(radians);
        double cos = Math.cos(radians);
        double forward_max_speed = Math.abs(cos);
        double forward_inches = cos * inches;
        double forward_direction_mult = (forward_inches > 0) ? 1.0 : -1.0;
        double sideways_max_speed = Math.abs(sin);
        double sideways_inches = sin * inches;
        double sideways_direction_mult = (sideways_inches > 0) ? 1.0 : -1.0;
        double target_tics = (forward_max_speed >= sideways_max_speed) ?
                (tics_per_inch_forward * forward_inches * forward_direction_mult) :
                (tics_per_inch_sideways * sideways_inches * sideways_direction_mult);
        while (true) {
            double current_tics = (forward_max_speed >= sideways_max_speed) ?
                    (forward_tics() * forward_direction_mult) : (sideways_tics() * sideways_direction_mult);
            if (current_tics > target_tics) {
                break;
            }
            double speed_mult = power_accel_decel(current_tics, target_tics, mtr_accel_min, mtr_decel_min,
                    mtr_accel_tics, mtr_decel_tics);
            set_speeds(forward_max_speed * speed_mult * forward_direction_mult,
                    sideways_max_speed * speed_mult * sideways_direction_mult, 0.0);
        }
        set_speeds(0.0, 0.0, 0.0);
    }

    /**
     * Rotate the robot by the specified number of degrees. Positive is clockwise
     * and negative is counter-clockwise.
     * @param degrees (double) The number of degrees to be rotated.
     */
    protected void rotate(double degrees) {
        double start_heading = heading;
        // Rotate as specified - there is normally some overshoot.
        turn(degrees);
        // Test the heading and correct for error (overshoot)
        turn(degrees - (heading - start_heading));
    }

    /**
     * Turn the specified number of degrees. This is a local function that
     * may have overshoot. Positive is clockwise and negative is counter-clockwise.
     * @param degrees (double) The number of degrees to be rotated.
     */
    protected void turn(double degrees) {
        reset_drive_encoders();

        double direction_mult = (degrees > 0.0) ? 1.0 : -1.0;
        double start_heading = heading;
        double target = degrees * direction_mult;
        while (true) {
            double current = direction_mult * (heading - start_heading);
            if (current >= target) {
                break;
            }
            set_speeds(0.0, 0.0,
                    direction_mult * power_accel_decel(current, target,
                            mtr_accel_min,mtr_decel_min,
                            mtr_accel_degs, mtr_decel_degs));
        }
        set_speeds(0.0, 0.0, 0.0);
    }

}