parameter gt is 33.
sas off.
rcs off.
lock throttle to 1.
stage.
when ship:altitude > 50000 then {
    ag1 on.
}
lock steering to "kill".
wait until ship:verticalspeed > gt. // 20
lock steering to heading(90, 85).
wait until vang(ship:facing:vector, heading(90, 85):vector) < 0.25.
lock steering to heading(90, 90 - vang(srfPrograde:vector, up:vector)).
set fs_sep to false.
until ship:obt:apoapsis > 97000 {
    if not fs_sep {
        if stage:resourceslex["LiquidFuel"]:amount < stage:resourceslex["LiquidFuel"]:capacity / 7 { 
            lock throttle to 0.
            wait 1.
            stage.
            //ag1 on.
            wait 3.
            stage.
            wait 0.5.
            lock throttle to 1.
            set fs_sep to true.
        }
    }
}
lock throttle to 0.25.
wait until ship:obt:apoapsis >= 100000.
lock throttle to 0.
wait until ship:altitude > 70000.
set mu to ship:body:mu.
set r to ship:obt:apoapsis + 600000.
set a1 to ship:obt:semimajoraxis.
set a2 to ship:obt:apoapsis + 600000.
set d1 to sqrt(mu*(2/r-1/a1)).
set d2 to sqrt(mu*(2/r-1/a2)).
set dv to d2 - d1.
set circu to node(time:seconds + eta:apoapsis, 0, 0, dv).
add circu.

set f to ship:availablethrust.
set eIsp to 0.
list engines in my_engines.
for eng in my_engines {
    set eISp to eIsp + eng:maxthrust / maxthrust * eng:isp.
}
set ve to eIsp * 9.81.
set m0 to ship:mass.
set m1 to m0 * constant:e ^ (-1 *circu:burnvector:mag / ve).
set ai to f / m0.
set af to f / m1.
set t to circu:burnvector:mag / ((ai + af) / 2).
set start_t to time:seconds + circu:eta - t / 2.
lock steering to circu:burnvector.
wait until vang(ship:facing:vector, circu:burnvector) < 0.25.
kuniverse:timewarp:warpto(start_t - 10).

wait until time:seconds >= start_t.
lock throttle to 1.
wait until circu:burnvector:mag <= 15.
lock throttle to 0.1.
wait until circu:burnvector:mag <= 0.2.
lock throttle to 0.
