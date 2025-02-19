package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumBase;

/**
 * Run an autonomous pattern.
 */
@Autonomous(name = "MecanumAutoExample", group = "")
public class MecanumAutoExample extends MecanumBase {

    /**
     * This function is executed when this Op Mode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() throws java.lang.InterruptedException {
        // initialize the mecanum base
        initialize_pre_start(hardwareMap);

        waitForStart();
        initialize_post_start();

        // run an autonomous pattern
        // Run in a 3' square
        move(36.0, 0.0);
        move(36.0, 90.0);
        move(-36.0, 0.0);
        move(36.0, -90.0);
        Thread.sleep(1000);

        // run in a 3' X within the above square
        move(50.91, 45.0);
        move(-36.0, 0.0);
        move(50.91, -45.0);
        move(-36.0, 0.0);
        Thread.sleep(1000);

        // spin (demonstrates crossing the -180 to/from 189 boundary
        rotate(360.0);
        rotate(-360.0);
    }
}