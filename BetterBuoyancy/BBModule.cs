﻿/*Copyright (c) 2014, Michael Ferrara
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
using System;
using System.Collections.Generic;
using UnityEngine;
using KSP;

namespace BetterBuoyancy
{
    public class BBModule : PartModule
    {
        bool moduleInitialized = false;

        const double VERT_CRASH_TOL_FACTOR = 1.2;
        const double HORIZ_CRASH_TOL_FACTOR = 7;
        const double OCEAN_DENSITY_IN_TONNE_PER_METER_CUBED = 1;

        private void FixedUpdate()
        {
            if (!FlightGlobals.ready || !vessel.mainBody.ocean || part.PhysicsSignificance == (int)Part.PhysicalSignificance.NONE || part.rigidbody == null)
                return;

            if (!moduleInitialized)
            {
                if (part.partBuoyancy == null)
                    return;
                GameObject.Destroy(part.partBuoyancy);
                moduleInitialized = true;
            }

            part.rigidbody.AddForceAtPosition((Vector3)BuoyancyForce(part.rb), part.transform.position, ForceMode.Force);
        }

        private Vector3d BuoyancyForce(Rigidbody body)
        {
            //We assume the ocean is at 0 altitude
            double depth = -FlightGlobals.getAltitudeAtPos((Vector3d)part.transform.position, vessel.mainBody);

            if (depth < 0)      //corresponds to above the ocean
            {
                if (part.WaterContact)
                {
                    part.WaterContact = false;
                    vessel.Splashed = false;
                    part.rigidbody.drag = 0;
                }
                return Vector3.zero;
            }

            if (!part.WaterContact)
            {
                part.WaterContact = true;
                vessel.Splashed = true;
            }
            ApplySplashEffect();

            if (CheckDieOnHighVelocity(body))
                return Vector3.zero;

            double vol;
            double depthForMaxForce;
            if (part.collider)
            {
                Vector3 size = part.collider.bounds.size;
                vol = size.magnitude;       //This is highly approximate, but it works
                depthForMaxForce = Math.Max(Math.Max(size.x, Math.Max(size.y, size.z)), 2);      //This is a very, very rough model of partial immersion
            }
            else
            {
                vol = 1;
                depthForMaxForce = 2;
            }

            double depthFactor = depth / depthForMaxForce;
            depthFactor = Math.Min(depthFactor, 1);

            part.rigidbody.drag = 3 * (float)depthFactor;

            Vector3d buoyancyForce = -FlightGlobals.getGeeForceAtPosition(part.transform.position) * depthFactor;
            buoyancyForce *= vol * OCEAN_DENSITY_IN_TONNE_PER_METER_CUBED;
            return buoyancyForce;
        }

        private void ApplySplashEffect()
        {
            float mag = rigidbody.velocity.magnitude * 0.1f;
            if(mag > 5)
                FXMonger.Splash(part.transform.position, mag);
        }

        private bool CheckDieOnHighVelocity(Rigidbody body)
        {
            Vector3d velVector = body.velocity + Krakensbane.GetFrameVelocityV3f();

            double vertVec = Vector3d.Dot(velVector, vessel.upAxis);
            if (Math.Abs(vertVec) > part.crashTolerance * VERT_CRASH_TOL_FACTOR)
            {
                GameEvents.onCrashSplashdown.Fire(new EventReport(FlightEvents.SPLASHDOWN_CRASH, part, part.partInfo.title, "", 0, ""));
                part.Die();
                return true;
            }
            double horizVel = (velVector - vertVec * vessel.upAxis).magnitude;
            if (horizVel > part.crashTolerance * HORIZ_CRASH_TOL_FACTOR)
            {
                GameEvents.onCrashSplashdown.Fire(new EventReport(FlightEvents.SPLASHDOWN_CRASH, part, part.partInfo.title, "", 0, ""));
                part.Die();
                return true;
            }
            return false;
        }
    }
}
