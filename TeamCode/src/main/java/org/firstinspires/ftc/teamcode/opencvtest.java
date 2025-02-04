package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous (name = "opencvtest", group = "Autonomous Main")
public class opencvtest extends LinearOpMode {

    Camera Webcam1 = new Camera(hardwareMap);
    @Override
    public void runOpMode(){
        Webcam1.getPipelineOutput();
        telemetry.addLine("press play");
        waitForStart();

        while (opModeIsActive()){
            telemetry.addLine(Webcam1.getPipelineOutput());
            telemetry.update();
            if (Webcam1.getPipelineOutput() == "Right"){
                telemetry.addLine("right");
            }
            if (Webcam1.getPipelineOutput() == "Left"){
                telemetry.addLine("left");

            }
            if (Webcam1.getPipelineOutput() == "Middle"){
                telemetry.addLine("middle");

            }

        }


    }

}