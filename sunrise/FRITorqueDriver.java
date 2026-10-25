// From RobotLocomotion/drake-iiwa-driver, kuka-driver/sunrise_1.14/DrakeFRITorqueDriver.java,
// renamed FRITorqueDriver (class and file; package drake_fri kept).
// Two changes for this cell, nothing else:
//   - client IP 192.170.10.200 -> 172.31.1.148 (iris, the address LBRServer's dialog uses);
//   - send period 5 ms -> 2 ms, the period lbr_fri_ros2 and the controllers (update_rate 500)
//     are tuned for.
// Joint impedance 1000/1000/1000/500/500/500/500, damping 1.0, TORQUE command mode and port 30200
// are Drake's. Install it with an applications-only sync (docs/sunrise_setup.md); Drake's README
// provisioning (StationSetup Install, unchecking Customer PSM rows 1-3) is NOT needed for it.
// With it running, sunrise_joint_stiffness must be the same seven numbers
// (docs/position_source.md).
package drake_fri;

// This file is loosely based on the KUKA sample code for the FRI interface.

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * Creates a FRI Session.
 */
public class FRITorqueDriver extends RoboticsAPIApplication {
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private int _clientPort;

    @Override
    public void initialize() {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "172.31.1.148";
        _clientPort = 30200;
    }

    private void doFRISession(FRIConfiguration friConfiguration) {
    	getLogger().info("Creating FRI connection to " + friConfiguration.getHostName() +
    			":" + friConfiguration.getPortOnRemote());
    	getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        // wait until FRI session is ready to switch to command mode
        try {
            friSession.await(3600, TimeUnit.SECONDS);
        } catch (final TimeoutException e) {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");
        FRIJointOverlay jointOverlay = new FRIJointOverlay(
            friSession, ClientCommandMode.TORQUE);

        JointImpedanceControlMode ctrMode =
            new JointImpedanceControlMode(1000, 1000, 1000, 500, 500, 500, 500);
        ctrMode.setDampingForAllJoints(1.0);
        PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);

        try {
            _lbr.move(posHold.addMotionOverlay(jointOverlay));
        } catch (final CommandInvalidException e) {
            getLogger().error(e.getLocalizedMessage());
        } finally {
            // done
            friSession.close();
        }
    }

    @Override
    public void run() {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(2);
        friConfiguration.setPortOnRemote(_clientPort);
        while (true) {
          doFRISession(friConfiguration);
        }
    }

    public static void main(final String[] args) {
        final FRITorqueDriver app = new FRITorqueDriver();
        app.runApplication();
    }
}
