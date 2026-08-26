# Sunrise controller setup for FRI

Windows-side procedure for installing the `LBRServer` FRI application on the
cabinet. Complete this before any step in
[`commissioning_protocol.md`](commissioning_protocol.md).

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
- That address must be one of the two in `client_names_` in `LBRServer.java`, or
  you edit that array. Stock values are `172.31.1.148` and `192.170.10.1`.
- The cabinet needs **no gateway and no internet route**. Sunrise.OS is not
  hardened for exposure to a general network, and nothing here requires the
  controller to reach anything but the ROS workstation.
- Exactly one default route may exist on the workstation, and it is not the
  robot one.

The simplest arrangement, and the one this workspace was commissioned on, is a
second address on the workstation's existing NIC rather than a controller-side
change:

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
A1-A7 referencing. It also means `LBRServer.java` runs unmodified.

For a second robot later, change the *new* cabinet during its own commissioning
rather than moving this one. Two robots on one ROS host also need distinct FRI
ports, distinct ROS namespaces, and distinct `robot_name` frame prefixes, and
stock `LBRServer.java` exposes no port selector.

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
listens while `LBRServer` is running on the smartPAD.

### A cautionary note on addresses found by guesswork

An early revision of this workspace recorded a cabinet address on the lab subnet,
identified by watching ARP while X66 was unplugged and replugged. It did not
reproduce: a full ARP sweep of that subnet with X66 connected and again with it
disconnected returned the same hosts in both states, and the recorded address
answered in neither. The controller was on the factory `172.31.1.0/24` subnet the
whole time.

Confirm against the controller itself — the smartPAD screen, Workbench's
connection, or a matched MAC. Do not carry forward an address that was inferred
rather than read.

## Debugging X66 reachability

Run this whenever the cabinet does not answer. It is ordered so that each step rules
out one layer. Substitute your own interface name for `<nic>` and your own subnets;
the addresses below are the KUKA factory defaults.

`arp-scan`, `nmap`, and `tcpdump` need root. `ping` does not.

### Do not use ping as the test

The cabinet does not answer ICMP echo requests. A silent `ping` means nothing at all,
in either direction, and reading it as "the robot is down" is how a wrong address
survives in a note for weeks.

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

**A red LED on the X66 port can be a working link.** Observed once: the port showed
red while the cabinet was booting, and the same port carried a confirmed ARP reply
from the controller minutes later with nothing recabled.

On many multi-gig switches the per-port LED encodes negotiated speed rather than
health — green for a 10G link, amber or red below it. The KLI negotiates well under
10G, so red is its normal steady state there. Check what your switch's LEDs actually
mean. Only a dark LED reliably means no link, and that is when to suspect the cable,
the X66 port, or a dead switch port.

### 6. Allow for boot time before concluding anything

The KLI comes up late in the Sunrise boot. A cabinet that is powered but still booting
is completely silent: no ARP reply, no link-up frame, nothing in a passive capture.
This has happened here: steps 2 through 4 all came back empty and the same scan
succeeded a few minutes later with no change to either end.

Before concluding that the robot is unreachable, confirm the smartPAD has finished
starting and re-run step 2.

## Tooling

**Sunrise.Workbench is required. WorkVisual is not a substitute.** On a Sunrise
cabinet WorkVisual is the fieldbus and I/O editor that Workbench launches for
`IOConfiguration.wvs`. It cannot create a Sunrise project, edit `StationSetup.cat`,
install option packages, or synchronize Java applications.

## Two corrections to the upstream guide

LBR-Stack's
[hardware setup guide](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html)
is otherwise accurate, but two of its steps do not apply here.

1. **`LBRServer.java` is not on the branch the guide tells you to clone.** The guide
   clones `fri-$FRI_CLIENT_VERSION`. In the 1.x line, `server_app/LBRServer.java`
   exists only on the `ros2-fri-1.15` branch of `lbr-stack/fri`; branches `fri-1.11`
   through `fri-1.17` contain the C++ client SDK and no `server_app/` directory.
   Take the Java file from `ros2-fri-1.15` regardless of the FRI version in use. The
   Sunrise FRI Java API is unchanged across these minor versions.
2. **The guide assumes a fresh project.** This cabinet already carries a project with
   a valid safety configuration and referencing state. Load that project from the
   controller instead of creating a new one.

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
   change does, and this cell has had one: the customer PSM table is deactivated so
   that AUT can be selected. Record the checksum from *after* that change as the
   baseline the later steps compare against, and see
   [`commissioning_protocol.md`](commissioning_protocol.md#deactivating-the-customer-psm-to-reach-aut)
   for what that change removed.

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

6. Create the application package: right-click `src` -> `New` -> `Package`, named
   exactly `lbr_fri_ros2`. The name must match `package lbr_fri_ros2;` on line 1 of
   `LBRServer.java` or the project will not build.

7. Copy in the server application:

   ```powershell
   git clone https://github.com/lbr-stack/fri.git -b ros2-fri-1.15 $HOME\Downloads\fri
   copy $HOME\Downloads\fri\server_app\LBRServer.java $HOME\SunriseWorkspace\<project>\src\lbr_fri_ros2\
   ```

   No edit to the file is required. `LBRServer.java` hardcodes
   `client_names_ = {"172.31.1.148", "192.170.10.1"}`, and `172.31.1.148` is the
   address the ROS workstation holds on the robot subnet. If your workstation uses
   a different address, that array is what must be edited.

8. Refresh `src` in Workbench, confirm `LBRServer` compiles, and synchronize
   applications. `LBRServer` should now appear in the smartPAD application list.

Running applications from the smartPAD may require `Safety maintenance technician`
rights, activated under `Safety` -> `Activation`.

## Startup order

`LBRServer.initialize()` runs four modal dialogs and then calls
`fri_session_.await(10, TimeUnit.SECONDS)`. This is a **10 second window**, not an
indefinite wait: on timeout the application logs the error and returns.

Start the ROS side first, then answer the dialogs:

```bash
pixi run -e jazzy hardware
```

Dialog answers for this workspace:

| Dialog | Answer |
| --- | --- |
| FRI send period [ms] | `2` |
| Remote IP address | `172.31.1.148` |
| FRI control mode | `JOINT_IMPEDANCE_CONTROL` |
| FRI client command mode | `TORQUE` |

## Open item: Sunrise joint stiffness

`LBRServer.java` hardcodes `new JointImpedanceControlMode(200, 200, 200, 200, 200,
200, 200)`, that is 200 Nm/rad on every axis. In `TORQUE` client command mode the
CRISP overlay is added on top of that impedance law, so Sunrise actively pulls back
toward the held position and works against `cartesian_impedance_controller`.

Run the first zero-overlay bringup at the stock 200 Nm/rad and decide from measured
behaviour. Lowering it toward zero means the arm is held only by the ROS-side torque
commands, which is a separate commissioning decision with its own safety review, not
a tuning tweak.

## Network caveat

A 2 ms FRI send period on a switch shared with general lab traffic is exposed to
jitter, and FRI drops the session on connection-quality degradation. If `LBRServer`
logs quality, jitter, or latency warnings, the fix is a dedicated link — a second NIC
or USB-Ethernet adapter in the workstation, cabled directly to X66 — not a longer
send period.
