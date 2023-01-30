package org.firstinspires.ftc.teamcode.Hardware;
//
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareProfile {
    /* Public OpMode members. */
    public DcMotor RearRightDrive;
    public DcMotor RearLeftDrive;
    public DcMotor FrontLeftDrive;
    public DcMotor FrontRightDrive;
    public DcMotor Carousel;
    public Servo ArmServo;
    public Servo IntakeLeft;
    public Servo IntakeRight;

    // Bot wheel and motor parameters
    private double COUNTS_PER_MOTOR_REV = 384.5;
    private double COUNTS_PER_CAROUSEL_REV = 384.5;
    private double DRIVE_GEAR_REDUCTION = 2.0;
    private double WHEEL_DIAMETER_INCHES = 3.779;
    private double CAROUSEL_WHEEL_DIAMETER = 4.75;
    public double CAROUSEL_COUNTS_PER_INCH = COUNTS_PER_CAROUSEL_REV / (CAROUSEL_WHEEL_DIAMETER * 3.1415);
    public double WHEEL_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareProfile() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        RearRightDrive = hwMap.get(DcMotor.class, "rearrightdrive");
        RearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        RearRightDrive.setPower(0);

        RearLeftDrive = hwMap.get(DcMotor.class, "rearleftdrive");
        RearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RearLeftDrive.setPower(0);

        FrontLeftDrive = hwMap.get(DcMotor.class, "frontleftdrive");
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setPower(0);

        FrontRightDrive = hwMap.get(DcMotor.class, "frontrightdrive");
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setPower(0);

        Carousel = hwMap.get(DcMotor.class, "Carousel");
        // Carousel.setDirection(DcMotor.Direction.REVERSE);
        Carousel.setTargetPosition(0);
        Carousel.setPower(0);
        Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Carousel.setPower(0);

        // define servo

        ArmServo = hwMap.get(Servo.class, "armservo");
        IntakeLeft = hwMap.get(Servo.class, "intakeleft");
        IntakeLeft.setDirection(Servo.Direction.FORWARD);
        IntakeRight = hwMap.get(Servo.class, "intakeright");
        IntakeRight.setDirection(Servo.Direction.REVERSE);


    }
}
