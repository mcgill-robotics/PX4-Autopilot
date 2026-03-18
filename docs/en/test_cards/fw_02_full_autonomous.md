# Test FW_02 - Full Autonomous

## Objective

To test the auto modes such as Mission, Loiter, and RTL for fixed wing vehicles.

## Preflight

Plan a mission on the ground. Ensure the mission has:

- Takeoff as first waypoint
- Changes in altitude throughout the mission
- At least one loiter waypoint
- Last waypoint is an RTL
- Duration of 5 to 6 minutes

## Flight Tests

❏ Mission

&nbsp;&nbsp;&nbsp;&nbsp;❏ Auto takeoff (hand launch or runway)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Changes in altitude throughout the mission

&nbsp;&nbsp;&nbsp;&nbsp;❏ First waypoint set to Takeoff

&nbsp;&nbsp;&nbsp;&nbsp;❏ Enable Mission End RTL

&nbsp;&nbsp;&nbsp;&nbsp;❏ Duration of 5 to 6 minutes

&nbsp;&nbsp;&nbsp;&nbsp;❏ Auto land or loiter at end

❏ Loiter

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage Loiter mode during manual flight

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle should orbit at current position and altitude

&nbsp;&nbsp;&nbsp;&nbsp;❏ Orbit radius and direction should match parameters

❏ RTL

&nbsp;&nbsp;&nbsp;&nbsp;❏ Arm and takeoff in any manual mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly out ~200m from start point

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage RTL Mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle should climb to RTL altitude if below it

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle should return to home and loiter or land

## Expected Results

- Mission should upload on first attempt
- Vehicle should automatically takeoff upon engaging Auto
- Waypoint tracking should be smooth with appropriate turn radius
- Vehicle should adjust height to RTL altitude before returning home
- Landing approach should be stable (if auto-land is configured)
