package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Scoring lift pos")
public class ScoringLiftPos extends LinearOpMode {

    float position = 0.5f;
    boolean positionToggle1 = true, positionToggle2 = true;
    private Servo scoringBucket;
    private DcMotor scoringLift;


    @Override
    public void runOpMode() {

        scoringBucket = hardwareMap.get(Servo.class, "bucket");
        scoringLift = hardwareMap.get(DcMotor.class, "scoringLift");

        waitForStart();
        scoringLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoringLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {

            telemetry.addData("Pos", scoringLift.getCurrentPosition());
            telemetry.update();
        }
    }
}
