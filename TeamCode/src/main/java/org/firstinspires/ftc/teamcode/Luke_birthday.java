package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(name="Luke_birthday")

public class Luke_birthday extends LinearOpMode {
    // names motors and sets motors to type null
    public DcMotor Frontright = null;
    public DcMotor Backright = null;
    public DcMotor Backleft = null;
    public DcMotor Frontleft = null;
    public Servo Claw = null;


    //sets booleans to false
    private final boolean isPressed = false;
    public boolean buttonPressed = false;
    //sets variables to 0
    public double lastTick = 0;
    public double lastTime = 0;
    public double currentTick = 0;
    public double currentTime = 0;
    public double targetShooterRPM = 0;


    //calls hardware map
    //pamh robot = new pamh();

    @Override
    public void runOpMode() {
        double speed;

        //sets up names for configuration
        Frontright = hardwareMap.get(DcMotor.class,"RF");
        Backright = hardwareMap.get(DcMotor.class,"RB");
        Backleft = hardwareMap.get(DcMotor.class,"LB");
        Frontleft = hardwareMap.get(DcMotor.class,"LF");
        Claw = hardwareMap.get(Servo.class,"C");

        // sets the right 2 motors to reverse
        Frontright.setDirection(DcMotor.Direction.REVERSE);
        Backright.setDirection(DcMotor.Direction.REVERSE);

        Frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Claw.setPosition(1);

        //calls from samplemecanumdrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //sets motors to run without encoders
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //initializes hardware map
        //robot.init(hardwareMap);

        waitForStart();



        while (opModeIsActive()){

            //sets all 4 base motors to the left and right joysticks on gamepad 1
            //uses the variables from SampleMecanumDrive to adjust motors
            //left stick in the y direction is for going forward and backward at 80% power
            //left stick in the x direction is for strafing left and right at 80% power
            //right stick in the x direction is for turning left and right at 80% power
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*1,
                            -gamepad1.left_stick_x*1,
                            -gamepad1.right_stick_x*0.8
                    )
            );

            drive.update();

            // Makes variables Power1 and Power2 to their respective joystick
            double Power1 = gamepad1.right_stick_y;
            double Power2 = gamepad1.left_stick_y;

            if (gamepad1.right_bumper) {
                Claw.setPosition(1);
            }
            //Closes claws when the left bumper on gamepad 2 is pressed
            else if (gamepad1.left_bumper){
                Claw.setPosition(0.7);
            }

            // sets speed depending on if the left trigger on gamepad 1 is pressed
            if (gamepad1.left_trigger > .1) {
                speed = .5;
            }
            else {
                speed = 1;
            }

            while(gamepad1.left_trigger > .1){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y/2,
                                -gamepad1.left_stick_x/2,
                                -gamepad1.right_stick_x/2
                        )
                );
            }

            //adds data to the driver hub that tells you the coordinates of where the robot is on the field
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("a", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            /*telemetry.addData("FrontDistance", FrontDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("FrontMagnet", FrontMagnet.getState());
            telemetry.addData("BackMagnet", BackMagnet.getState());*/
            telemetry.update();
            }
        }

    }
// skye is sleepy
//angry birds pig
