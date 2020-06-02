// Copyright 2020 tony48
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO 
// THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// Original C# functions by djungelorm

//runpath("lib_nav.ks").
clearscreen.
sas off.
// on commence par faire la correction d'inclinaison
set dn to addons:utils:TrueAnomalyAtDN(ship, target). // on trouve le dn
set an to addons:utils:TrueAnomalyAtAN(ship, target). // on trouve l'an
set dnUT to addons:utils:UTAtTrueAnomaly(ship, dn). // le temps du dn
set anUT to addons:utils:UTAtTrueAnomaly(ship, an). // le temps de l'an
set relInc to addons:utils:RelativeInclination(ship, target). // l'inclinaison relative entre nous et la target
if relInc > 0.09 { // si l'inclinaison est importante on corrige
    set dvI to 2 * velocityAt(ship, dnUT):orbit:mag * sin(relInc/2). // le dv de la maneuvre
    if dnUT < anUT { // on prend le premier node
        set incCorrection to node(dnUT, 0, dvI, 0).
    } else {
        set incCorrection to node(anUT, 0, -dvI, 0). // si on est a l'AN il faut burn antinormal, donc -dvI
    }
    add incCorrection.
    executeNode(). // et on fait la correction
}

// ici on fait le transfer de hohmann
set tranferSma to (target:obt:semimajoraxis + ship:obt:semimajoraxis) / 2. // le sma du transfer, ca devrait etre clair
set transferTime to constant:pi * sqrt(tranferSma^3 / ship:body:mu). // le temps du transfert qui correspond a la moitie de l'orbite de transfert
set shpObtSpeed to 360 / ship:obt:period. // notre vitesse angulaire
set tgtObtSpeed to 360 / target:obt:period. // la vitesse angulaire de la tgt
set pa to 180 - transferTime * tgtObtSpeed. // l'angle de phase
set angle1 to ship:obt:lan + ship:obt:argumentofperiapsis + ship:obt:trueanomaly. // l'angle actuel de nous
set angle2 to target:obt:lan + target:obt:argumentofperiapsis + target:obt:trueanomaly. // l'angle actuel de la tgt
set angle3 to angle2 - angle1. // l'angle actuel entre la tgt et nous
set angle3 to angle3 - 360 * floor(angle3/360). // on s'assure que ca depasse pas 360
set paRoC to tgtObtSpeed - shpObtSpeed. // la vitesse a laquelle l'angle change
set dang to (angle3 - pa) - 360 * floor((angle3 - pa)/360). // la difference entre l'angle cible et l'angle actuel
set timeToTransfer to abs(dang/paRoC). // le temps entre maintenant et la maneuvre
// un bon coup de vis viva
set r to (positionAt(ship, timeToTransfer + time:seconds) - ship:body:position):mag. // le rayon du vaisseau a la maneuvre
set tgtPositionRdv to (positionAt(target, timeToTransfer + transferTime + time:seconds) - ship:body:position):mag. // la position de la tgt a l'apo de l'hohmann
set a to (r + tgtPositionRdv) / 2. // le sma du tranfer (plus precis maintenant)
set vh to sqrt(ship:body:mu*(2/r-1/a)). // la vitesse cible a r
set va to sqrt(ship:body:mu*(2/r-1/ship:obt:semimajoraxis)). // lavitesse prévu a r
set dvh to vh - va. // le dv de la maneuvre, vitesse cible - vitesse prévu
set hohmann to node(time:seconds + timeToTransfer, 0, 0, dvh).
add hohmann. // on ajoute un node
until tgtPositionRdv >= hohmann:obt:apoapsis + 600000 { // l'hohmann peut produire deux rencontres et on en veut qu'une seule
    set hohmann:prograde to hohmann:prograde - 0.1.
}


//set ap to (positionAt(target, timeToTransfer + transferTime + time:seconds) - positionAt(ship, timeToTransfer + transferTime + time:seconds)):mag.
set ap to (positionAt(target, hohmann:eta + hohmann:obt:period / 2 + time:seconds) - positionAt(ship, hohmann:eta + hohmann:obt:period / 2 + time:seconds)):mag.
print ap.
set hohmann:eta to hohmann:eta + 0.1.
set ap1 to apCalc().
if ap1 < ap { // en gros on change le temps de la maneuvre jusqu'a obtenir la rencontre la plus proche
    set ap to ap1.
    until ap1 > ap {
        set ap to ap1.
        set hohmann:eta to hohmann:eta + 0.1.
        set ap1 to apCalc().
    }
    set hohmann:eta to hohmann:eta - 0.1.
} else if ap1 > ap {
    set ap to ap1.
    until ap1 > ap {
        set ap to ap1.
        set hohmann:eta to hohmann:eta - 0.1.
        set ap1 to apCalc().
    }
    set hohmann:eta to hohmann:eta + 0.1.
}
executeNode(). // on execute le node
set cas to ListClosestApproaches(target, 1). // le nombre d'approches (normalement une)
set altca to (positionAt(ship, cas[0][0]) - ship:body:position):mag. // notre altitude a l'approche
// vis viva pour rel speed
//set va to sqrt(ship:body:mu*(2/altca-1/ship:obt:semimajoraxis)).
//set vc to sqrt(ship:body:mu*(2/altca-1/altca)).
//set dvca to vc - va.
set va to velocityAt(ship, cas[0][0]):orbit:mag.
set vc to velocityAt(target, cas[0][0]):orbit:mag.
set dvca to vc - va.
set canode to node(cas[0][0] - 3, 0, 0, dvca).
add canode. // node pour annuler la vitesse relative
executeNode2(). // on l'execute mais pas avec la meme methode

lock relativeVelocityVec to target:velocity:orbit - ship:velocity:orbit. // le vecteur de velocite relative
lock relativeSpeed to relativeVelocityVec:mag. // la vitesse relative
//lock steering to relativeVelocityVec. // on pointe retrograde par rapport au target
//wait until vang(relativeVelocityVec, ship:facing:vector) < 0.5.
//lock throttle to 0.5. // on annule la vitesse
//wait until relativeSpeed < 1.
//lock throttle to 0.1.
//wait until relativeSpeed < 0.1.
//lock throttle to 0.
lock steering to target:direction.
wait until vang(target:direction:vector, ship:facing:vector) < 0.5.
lock throttle to 0.2.
wait until relativeSpeed >= 5. // on se rapproche de la cible
lock throttle to 0.
lock steering to relativeVelocityVec.
wait until vang(relativeVelocityVec, ship:facing:vector) < 0.5.
wait until target:distance < 50.
lock throttle to 0.2.
wait until relativeSpeed < 0.1.
lock throttle to 0.
lock steering to "kill".
set tn to target:name.
print "Please select a docking port".
wait until target:name <> tn. // IMPORTANT le script attend qu'on selectionne un docking port
runpath("dock.ks"). // et on se dock, fin du script


function apCalc {
    return (positionAt(target, hohmann:eta + hohmann:obt:period / 2 + time:seconds) - positionAt(ship, hohmann:eta + hohmann:obt:period / 2 + time:seconds)):mag.
}


function executeNode2 {
    local nd to nextNode.
    local eIsp to 0.
    local my_engines to list().
    list engines in my_engines.
    for eng in my_engines {
        set eIsp to eIsp + eng:maxThrust / ship:maxthrust * eng:isp.
    }
    local ve to eIsp * 9.82.
    local m0 to ship:mass.
    local m1 to m0 * constant:e ^ (-1 *nd:burnvector:mag / ve).
    local ai to ship:availablethrust / ship:mass.
    local af to ship:availablethrust / m1.
    local t to nd:burnvector:mag / ((ai + af) / 2).
    local start_t to time:seconds + nd:eta - t.

    lock relativeVelocityVec to target:velocity:orbit - ship:velocity:orbit.
    lock steering to relativeVelocityVec.
    wait until vang(ship:facing:vector, relativeVelocityVec) < 0.5.
    kuniverse:timewarp:warpto(start_t - 10).

    wait until time:seconds >= start_t.
    lock throttle to 1.
    wait until relativeVelocityVec:mag <= 15.
    lock throttle to 0.3.
    wait until relativeVelocityVec:mag <= 0.2.
    lock throttle to 0.
    lock steering to prograde.
    remove nd.
}

function executeNode {
    local nd to nextNode.
    local eIsp to 0.
    local my_engines to list().
    list engines in my_engines.
    for eng in my_engines {
        set eIsp to eIsp + eng:maxThrust / ship:maxthrust * eng:isp.
    }
    local ve to eIsp * 9.82.
    local m0 to ship:mass.
    local m1 to m0 * constant:e ^ (-1 *nd:burnvector:mag / ve).
    local ai to ship:availablethrust / ship:mass.
    local af to ship:availablethrust / m1.
    local t to nd:burnvector:mag / ((ai + af) / 2).
    local start_t to time:seconds + nd:eta - t / 2.

    lock steering to nd:burnvector.
    wait until vang(ship:facing:vector, nd:burnvector) < 0.5.
    kuniverse:timewarp:warpto(start_t - 10).

    wait until time:seconds >= start_t.
    lock throttle to 1.
    wait until nd:burnvector:mag <= 15.
    lock throttle to 0.3.
    wait until nd:burnvector:mag <= 0.2.
    lock throttle to 0.
    lock steering to prograde.
    remove nd.
}

function CalcClosestApproach {
    parameter shp, tgt, begintime.
    set approachTime to begintime.
    set approachDistance to 100000000000.
    set minTime to begintime.
    set interval to shp:obt:period.
    //if myOrbit:eccentricity > 1 {
    //    set interval to 100 / 
    //}
    set maxTime to minTime + interval.


    set timeStep to (maxTime - minTime) / 20.
    set placeholder to minTime.
    until placeholder >= maxTime {
        set PosA to positionAt(shp, placeholder).
        set PosB to positionAt(tgt, placeholder).
        set thisDistance to (PosA - PosB):mag.
        if thisDistance < approachDistance {
            set approachDistance to thisDistance.
            set approachTime to placeholder.
        }
        set placeholder to placeholder + timeStep.
    }

    set fineMinTime to approachTime - timeStep.
    set fineMaxTime to approachTime + timeStep.
    if fineMaxTime > maxTime {
        set fineMaxTime to maxTime.
    }
    if fineMinTime < minTime {
        set fineMinTime to minTime.
    }
    set timeStep to (fineMaxTime - fineMaxTime / 50).
    set placeholder to fineMinTime.

    until placeholder >= fineMaxTime {
        set PosA to positionAt(shp, placeholder).
        set PosB to positionAt(tgt, placeholder).
        set thisDistance to (PosA - PosB):mag.
        if thisDistance < approachDistance {
            set approachDistance to thisDistance.
            set approachTime to placeholder.
        }
        set placeholder to placeholder + timeStep.
    }
    return list(approachTime, approachDistance).
}

function ListClosestApproaches {
    parameter tgt, orbits.
    set times to list().
    set distances to list().
    //set distance to 0.
    set orbitStart to time:seconds.
    set period to ship:obt:period.
    from {local i is 0. } until i >= orbits step {set i to i + 1.} do {
        set ca to CalcClosestApproach(ship, tgt, orbitStart).
        times:add(ca[0]).
        distances:add(ca[1]).
        set orbitStart to orbitStart + period.
    }
    set combined to list(times, distances).
    return combined.
}