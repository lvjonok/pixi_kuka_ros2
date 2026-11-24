# Sunrise controller setup for FRI

Windows-side procedure for installing the FRI application
[`sunrise/FRITorqueDriver.java`](../sunrise/FRITorqueDriver.java) on the cabinet. Complete this before
any step in [`commissioning_protocol.md`](commissioning_protocol.md).

## Versions to match

- Robot: KUKA LBR iiwa 14 R820, kinematic system `K1`
- Sunrise.OS on the cabinet: read it from the smartPAD
- Sunrise.Workbench on the Windows PC: must match the cabinet's Sunrise.OS
- FRI client SDK to build against: the FRI version installed in Sunrise
- FRI port: UDP `30200`

A Workbench newer than the controller installs its own OS version as a side
effect of any `StationSetup.cat` install, which invalidates the safety acceptance
and forces the A1-A7 referencing to be redone. Confirm the two match before
installing anything.

## Network layout

**Finding the cabinet's address is your job, and it is the first thing to do.**
Read it from the smartPAD network configuration screen, or from Workbench's
connection dialog. `172.31.1.147` is the KUKA factory default for X66 (KLI) and is
what an untouched cabinet answers on, but treat any address as provisional until
the controller itself confirms it.

What this workspace needs, whatever the addresses turn out to be:

- The ROS workstation must have an address on the **same subnet as X66**, on the
  interface cabled to it.
- That address must be the client address in `FRITorqueDriver.java`, `172.31.1.148`, or you edit
  `_clientName` there.
- The cabinet needs **no gateway and no internet route**. Sunrise.OS is not
  hardened for exposure to a general network, and nothing here requires the
  controller to reach anything but the ROS workstation.
- Exactly one default route may exist on the workstation, and it is not the
  robot one.

The simplest arrangement, and the one this cell uses, is a second address on the
workstation's existing NIC rather than a controller-side change:

```text
workstation  <nic> --+
                     +-- switch ---- router -- internet
KUKA X66 ------------+

<nic>:
  <lab-address>/24     gateway <lab-gateway>   lab network and internet
  172.31.1.148/24      no gateway              robot
```

Add the second address with:

```bash
nmcli con mod "<connection-name>" +ipv4.addresses 172.31.1.148/24
nmcli con up "<connection-name>"
```

Keeping the cabinet on its factory address avoids a `StationSetup.cat` install,
and therefore a reboot and a re-verification of the safety configuration and the
A1-A7 referencing. It also means `FRITorqueDriver.java` needs no edit.

### Record the MAC, not just the address

Whatever address you find, record the cabinet's **MAC** alongside it. The MAC is
the durable identifier: it survives an address change, and it is what separates
the real cabinet from another host that happens to answer on that address.

Kontron builds the embedded PC inside the Sunrise Cabinet, so a Kontron OUI on the
robot subnet identifies the controller rather than some other machine. A TCP scan
of a Sunrise controller returns `139/netbios-ssn`, `445/microsoft-ds`, and
`3389/ms-wbt-server` open with everything else closed, which distinguishes it from
the Linux hosts on a lab subnet even if the MAC is unknown.

`30200/tcp` reads as closed and that is correct: FRI is UDP, and the port only
listens while an FRI application is running on the smartPAD.

### Read the address, do not infer it

Confirm the address against the controller itself: the smartPAD screen, Workbench's
connection, or a matched MAC. An address inferred from watching ARP while X66 is unplugged
and replugged can belong to another host.

## Debugging X66 reachability

Run this whenever the cabinet does not answer. It is ordered so that each step rules
out one layer. Substitute your own interface name for `<nic>` and your own subnets;
the addresses below are the KUKA factory defaults.

`arp-scan`, `nmap`, and `tcpdump` need root. `ping` does not.

### Do not use ping as the test

The cabinet does not answer ICMP echo requests. A silent `ping` means nothing at all,
in either direction.

Test with ARP instead. ARP is answered by the NIC itself, below any firewall, so it
reports whether the controller is physically on the wire regardless of what Sunrise is
doing above it.

### 1. Check this end first

```bash
ip -br addr show <nic>         # expect the robot-subnet address on this interface
ip route get 172.31.1.147      # expect: dev <nic> src 172.31.1.148
ethtool <nic> | grep -E 'Speed|Link detected'
```

If `ip route get` names any interface other than the one cabled to X66, the second
address is missing; re-apply it with the `nmcli` command under
[Network layout](#network-layout). A workstation with Wi-Fi on the same subnet as its
Ethernet will route the wrong way, so confirm the route rather than assuming it.

### 2. ARP scan the robot subnet

```bash
sudo arp-scan -I <nic> 172.31.1.0/24
```

One host, with the MAC you recorded, means the cabinet is up and the address is
unchanged. Stop here.

### 3. ARP scan the lab subnet

```bash
sudo arp-scan -I <nic> -s <lab-address> <lab-subnet>
```

Only relevant if step 2 was empty: it catches a cabinet whose KLI was reconfigured
onto the lab subnet. Look for a Kontron OUI, not for a particular address. Linux
hosts answer `22/tcp`; a Sunrise controller answers `139`, `445`, and `3389` instead,
which distinguishes it even if the MAC is unknown:

```bash
sudo nmap -n -Pn -sS --top-ports 25 --open -e <nic> <lab-subnet>
```

### 4. Watch the wire passively

```bash
sudo tcpdump -i <nic> -n -e -p not host <lab-address>
```

Leave it running and unplug, then replug, the cable at the **X66 end**. A booted KLI
emits gratuitous ARP on link-up. If no frame ever arrives from an unknown MAC, nothing
is transmitting and the problem is physical or the cabinet is not finished booting.

### 5. Read the switch LED correctly

**A red LED on the X66 port can be a working link.** A port that shows red while the
cabinet boots can carry the controller's ARP reply minutes later with nothing recabled.

On many multi-gig switches the per-port LED encodes negotiated speed rather than
health — green for a 10G link, amber or red below it. The KLI negotiates well under
10G, so red is its normal steady state there. Check what your switch's LEDs actually
mean. Only a dark LED reliably means no link, and that is when to suspect the cable,
the X66 port, or a dead switch port.

### 6. Allow for boot time before concluding anything

The KLI comes up late in the Sunrise boot. A cabinet that is powered but still booting
is completely silent: no ARP reply, no link-up frame, nothing in a passive capture.
Steps 2 through 4 can all come back empty and the same scan succeed a few minutes later
with no change to either end.

Before concluding that the robot is unreachable, confirm the smartPAD has finished
starting and re-run step 2.

## Tooling

**Sunrise.Workbench is required. WorkVisual is not a substitute.** On a Sunrise
cabinet WorkVisual is the fieldbus and I/O editor that Workbench launches for
`IOConfiguration.wvs`. It cannot create a Sunrise project, edit `StationSetup.cat`,
install option packages, or synchronize Java applications.

## The upstream guide

LBR-Stack's
[hardware setup guide](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html)
covers the installation of FRI on the cabinet. It assumes a fresh project. This cabinet already
carries a project with a valid safety configuration and referencing state: load that project from
the controller instead of creating a new one.

## Procedure

1. Point Sunrise.Workbench at the cabinet and confirm the address it connects on.
   Record it; it supersedes anything written above.

2. Open `File` -> `New` -> `Sunrise project`, enter the controller address, and
   retrieve the project already on the controller. Do not create a new project.
   Archive an untouched copy of the retrieved project outside
   `%USERPROFILE%\SunriseWorkspace` before editing anything.

3. Record the pre-change state: the safety configuration checksum from the smartPAD
   (`Safety` -> `Status`), and the installed option packages from
   `StationSetup.cat` -> `Software`.

   Installing FRI must not change the checksum. A deliberate safety configuration
   change does. On this cell the customer PSM table is deactivated so that AUT can be
   selected: the checksum of that configuration is the baseline the later steps compare
   against, and
   [`commissioning_protocol.md`](commissioning_protocol.md#deactivating-the-customer-psm-to-reach-aut)
   says what that configuration leaves out.

4. Determine whether FRI is already installed. If the smartPAD application list
   contains `LBRJointSineOverlay`, `LBRTorqueSineOverlay`, and
   `LBRWrenchSineOverlay`, the Fast Robot Interface package is present and licensed,
   and step 5 can be skipped entirely. Do not run the torque or wrench examples;
   they move the robot.

5. Only if FRI is absent: add `Fast Robot Interface` in `StationSetup.cat` ->
   `Software`, then `Install` -> `Save and apply`, confirm the reboot, and
   synchronize applications after restart. Compare the safety checksum against
   step 3 afterwards, and re-check that the A1-A7 position-reference warning is
   still cleared.

6. If the project has no package `lbr_fri_ros2`, create it: right-click `src` -> `New` ->
   `Package`, named exactly `lbr_fri_ros2`. The name must match `package lbr_fri_ros2;` in
   `FRITorqueDriver.java` or the project will not build.

7. Copy [`sunrise/FRITorqueDriver.java`](../sunrise/FRITorqueDriver.java) of this repo into
   `src\lbr_fri_ros2\` of the project. No edit to the file is required: its client is
   `172.31.1.148`, the address the ROS workstation holds on the robot subnet, on port `30200`. If
   your workstation uses a different address, `_clientName` is what must be edited.

8. Refresh `src` in Workbench, confirm `FRITorqueDriver` compiles, and **synchronize
   applications**. Nothing else: no `StationSetup.cat` install, no option package, no safety
   configuration edit, no reboot. `FRITorqueDriver` should now appear in the smartPAD
   application list.

9. Read the safety checksum on the smartPAD again. It must equal step 3's. If it does not, stop
   and restore the archived project; do not run the application.

Running applications from the smartPAD may require `Safety maintenance technician`
rights, activated under `Safety` -> `Activation`.

**Do not copy Drake's provisioning.** `FRITorqueDriver.java` is derived from drake-iiwa-driver,
whose README presses `Install` in `StationSetup.cat` (which wipes and reboots the cabinet) and
unchecks External EMERGENCY STOP, Operator Protection and Safety Stop in the Customer PSM. None of
that is needed for this application and none of it may be done on this cell: the application goes
in by the sync above.

## Starting it

`FRITorqueDriver` asks nothing: it has no dialogs. Its values are in the file: client
`172.31.1.148` on port `30200`, send period 2 ms, the `TORQUE` client command mode.

Select `FRITorqueDriver` in the smartPAD application list and start it. It opens an FRI session
and waits for the client, an hour at a time (`friSession.await(3600, TimeUnit.SECONDS)`); on a
timeout it closes the session and opens the next, so it waits for as long as it runs. The ROS
side may be started before or after it:

```bash
pixi run -e jazzy hardware
```

With the client connected it holds position under its joint impedance with the FRI torque
overlay. When that motion returns, the session is closed and the loop in `run()` opens the next
one:

- A restart of `hardware` brings the FRI session back with nobody at the smartPAD. With the
  application left running, a launch on the PC is all it takes to put the arm under FRI: the
  list [before FRI starts](commissioning_protocol.md#before-fri-starts) is gone through before
  every launch.
- The application has no end of its own. It runs until it is stopped on the smartPAD.

## Sunrise joint stiffness

`FRITorqueDriver.java` runs `new JointImpedanceControlMode(1000, 1000, 1000, 500, 500, 500, 500)`
with `setDampingForAllJoints(1.0)`: 1000 Nm/rad on A1 to A3, 500 on A4 to A7, damping 1.0. In
`TORQUE` client command mode the torque overlay is added on top of that impedance law, around the
position FRI carries. At REST that position is fixed and the spring holds the arm. ARMED it is
the IK of crisp's target, held within a bounded gap of the arm, and the spring pulls toward the
target with a bounded torque ([`position_source.md`](position_source.md)).

FRI does not report the stiffness. `sunrise_joint_stiffness` of
`fri_position_passthrough_controller` is written for this application, and the rule for it is in
[`position_source.md`](position_source.md#the-sunrise-application-and-sunrise_joint_stiffness).
The damping is not a parameter of the stack: it is set in the Java file only.

Lowering the stiffness toward zero means the arm is held only by the ROS-side torque
commands, which is a separate commissioning decision with its own safety review, not a
tuning tweak.

Checked on the arm, 2026-10-25, at REST: the external torque Sunrise estimates is 1000 Nm/rad on
A1 to A3 and 500 on A4 to A7 times the distance of the joint from its commanded position.

## The other file in `sunrise/`

[`sunrise/LBRServerJointImpedance.java`](../sunrise/LBRServerJointImpedance.java) is a variant of
lbr_fri_ros2_stack's `LBRServer` that asks for a stiffness and a damping preset in two more
dialogs. It has not been compiled, and nothing here is written for it.

## Network caveat

A 2 ms FRI send period on a switch shared with general lab traffic is exposed to
jitter, and FRI drops the session on connection-quality degradation. `FRITorqueDriver` does not
log the connection quality: a dropped session shows on the ROS side, where the driver logs
`LBR left COMMANDING_ACTIVE` and closes its FRI client. The fix is a dedicated link (a second NIC
or USB-Ethernet adapter in the workstation, cabled directly to X66), not a longer send period.
