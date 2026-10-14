/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.Test.benchmarking;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.TreeMap;

@Disabled
@TeleOp(group="3", name="Telemetry POST Request Test")
public class TelemetryPOSTRequestTest extends LinearOpMode {
    private ElapsedTime lynxTimer = new ElapsedTime();
    public static double telemetry1 = 100;
    public static TreeMap<Double, Double> telemetry2 = new TreeMap<>();

    @Override
    public void runOpMode() {
        for (double i = 0; i < 100000; i++)
        {
            telemetry2.put(i, i);
        }

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()){
                lynxTimer.reset();
                telemetry.addData("telemetry1", telemetry1);
                double time = lynxTimer.milliseconds();
                telemetry.addData("Time to send telemetry packet 1 in ms: ", time);
                lynxTimer.reset();
                telemetry.addData("telemetry1", telemetry1);
                time = lynxTimer.milliseconds();
                telemetry.addData("Time to send telemetry packet 2 in ms: ", time);
            }

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
}
