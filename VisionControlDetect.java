package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Point;

import java.util.*;

@TeleOp(name = "视觉对齐色块", group = "Vision")
public class VisionControlDetect extends LinearOpMode {

    OpenCvWebcam webcam;
    ColorDetectionPipeline pipeline;

    DcMotor frontLeft, frontRight, backLeft, backRight;
    Servo Arm, ClawTurn, ClawOpen, Small;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ColorDetectionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        frontLeft = hardwareMap.get(DcMotor.class, "leftfront");
        frontRight = hardwareMap.get(DcMotor.class, "rightfront");
        backLeft = hardwareMap.get(DcMotor.class, "leftback");
        backRight = hardwareMap.get(DcMotor.class, "rightback");
        Arm = hardwareMap.get(Servo.class, "Big");
        ClawTurn = hardwareMap.get(Servo.class, "clawturn");
        ClawOpen = hardwareMap.get(Servo.class, "clawopen");
        Small = hardwareMap.get(Servo.class, "Small");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        Arm.setPosition(0);
        sleep(100);
        Small.setPosition(1);
        sleep(100);
        Arm.setPosition(1);
        sleep(500);
        ClawOpen.setPosition(0.5);
        sleep(500);

        telemetry.addLine("等待启动...");
        telemetry.update();
        waitForStart();

        Point imageCenter = new Point(84, 38);
        double kP = 0.015;
        Point lockedTarget = null;

        while (opModeIsActive()) {
            List<Point> redPoints = new ArrayList<>(pipeline.redCenters);
            Point currentTarget = null;

            if (lockedTarget != null && redPoints.contains(lockedTarget)) {
                currentTarget = lockedTarget;
                telemetry.addLine("目标保持锁定");
            } else if (!redPoints.isEmpty()) {
                redPoints.sort(Comparator.comparingDouble(p ->
                        Math.pow(p.x - imageCenter.x, 2) + Math.pow(p.y - imageCenter.y, 2)));
                currentTarget = redPoints.get(0);
                lockedTarget = currentTarget;
                telemetry.addLine("锁定新目标");
            } else {
                lockedTarget = null;
                telemetry.addLine("未检测到色块");
            }

            if (currentTarget != null) {
                // 假设 redPoints 至少有两个点用于计算中心线
                if (redPoints.size() >= 2) {
                    Point p1 = redPoints.get(0);  // 长方体的一个点
                    Point p2 = redPoints.get(1);  // 对角点

                    // 计算中心线的倾斜角度
                    double deltaX = p2.x - p1.x;
                    double deltaY = p2.y - p1.y;
                    double angle = Math.atan2(deltaY, deltaX);  // 使用 atan2 来计算角度（弧度）

                    // 转换为度数
                    double angleDegrees = Math.toDegrees(angle);

                    // 调整爪子根据角度旋转
                    double clawTurnAngle = angleDegrees / 90.0;  // 假设 -90 到 +90 映射为 0 到 1
                    ClawTurn.setPosition(clawTurnAngle);  // 设置爪子角度

                    telemetry.addData("倾斜角度", angleDegrees);
                }

                double dx = currentTarget.x - imageCenter.x;
                double dy = currentTarget.y - imageCenter.y;
                double distanceSquared = dx * dx + dy * dy;
                double threshold = 5;

                if (distanceSquared < threshold * threshold) {
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    telemetry.addLine("已对齐目标点");

                    Small.setPosition(0);
                    Arm.setPosition(0.2);
                    sleep(500);
                    ClawOpen.setPosition(0.5);
                    sleep(250);
                    ClawOpen.setPosition(1);
                    sleep(500);
                    Small.setPosition(1);
                    sleep(100);
                    Arm.setPosition(1);
                    sleep(500);
                    ClawOpen.setPosition(0.5);
                    lockedTarget = null;
                } else {
                    double xPower = dx * kP;
                    double yPower = -dy * kP;

                    xPower = Math.max(-1.0, Math.min(1.0, xPower));
                    yPower = Math.max(-1.0, Math.min(1.0, yPower));

                    double denominator = Math.max(Math.abs(yPower) + Math.abs(xPower), 1);
                    double flPower = (yPower + xPower) / denominator;
                    double frPower = (yPower - xPower) / denominator;
                    double blPower = (yPower - xPower) / denominator;
                    double brPower = (yPower + xPower) / denominator;

                    frontLeft.setPower(flPower);
                    frontRight.setPower(frPower);
                    backLeft.setPower(blPower);
                    backRight.setPower(brPower);

                    telemetry.addData("dx", dx);
                    telemetry.addData("dy", dy);
                }
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            telemetry.update();
            sleep(20);
        }
    }
}
