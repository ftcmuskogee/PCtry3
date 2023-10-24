package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Core;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import java.util.List;

@Autonomous (name = "visiontry", group = "Autonomous Main")
public class visiontry extends LinearOpMode {

    OpenCvWebcam webcam = null;

    @Override
    public void runOpMode() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        webcam.setPipeline(Pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //figure out ater
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        class Pipeline extends OpenCvPipeline {

            Mat YCbCr = new Mat();
            // gotta use rgb thing from centerstage instead
            Mat left;
            Mat center;
            Mat right;
            double leftavgfin;
            double centeravgfin;
            double rightavgfin;
            Mat output = new Mat();
            Scalar rectColor = new Scalar();

            public Mat proccessframe(Mat input) {
                Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_YCbCr);
                telemetry.addLine("pipelinerunning");

                // divide width in 3
                Rect leftRect = new Rect(1, 1, 213, 480);
                Rect centerRect = new Rect(213, 1, 213, 480);
                Rect rightRect = new Rect(316, 1, 213, 480);

                //colors in image
                input.copyTo(output);
                Imgproc.rectangle(output, leftRect, rectColor, 2);
                Imgproc.rectangle(output, centerRect, rectColor, 2);
                Imgproc.rectangle(output, rightRect, rectColor, 2);

                // mini map per frame
                left = YCbCr.submat(leftRect);
                center = YCbCr.submat(centerRect);
                right = YCbCr.submat(rightRect);

                //remove all colors but the one specified
                // get image
                //chanel 2 is red in YCbCr so
                Core.extractChannel(left, left, 2);
                Core.extractChannel(center, center, 2);
                Core.extractChannel(right, right, 2);

                Scalar leftavg = Core.mean(left);
                Scalar centeravg = Core.mean(center);
                Scalar rightavg = Core.mean(right);

                leftavgfin = leftavg.val[0];
                rightavgfin = rightavg.val[0];
                centeravgfin = centeravg.val[0];


                if (leftavgfin > rightavgfin) {
                    if (leftavgfin > centeravgfin) {
                        telemetry.addLine("left");

                    }

                    if (rightavgfin > leftavgfin) {
                        telemetry.addLine("right");
                    }

                else{
                        telemetry.addLine("center");
                    }
                    return (output);

                }

            }
        }

    }
}

// gotta find correct values for things
// probably set up center/right/left like example