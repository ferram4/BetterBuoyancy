/*Copyright (c) 2015, Michael Ferrara
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

        double vertCrashTolFactor = 1.2;
        double horizCrashTolFactor = 7;
        double overrideVol = -1;
        double overridedepthForMaxForce = -1;

        public void SetOverrideParams(double volume, double fullImmersionDepth)
        {
            overrideVol = volume;
            overridedepthForMaxForce = fullImmersionDepth;
        }

        public void SetCrashFactors(double vertCrashTolFactor, double horizCrashTolFactor)
        {
            this.vertCrashTolFactor = vertCrashTolFactor;
            this.horizCrashTolFactor = horizCrashTolFactor;
        }

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

            double depth = CalculateDepth((Vector3d)part.transform.position);
            UpdateWaterContact(depth);
            if (depth >= 0)
            {
                ApplyBouyancyForce(BuoyancyForce(part.rb, depth));
            }
        }

        protected virtual void ApplyBouyancyForce(Vector3d buoyancyForce)
        {
            part.rigidbody.AddForceAtPosition((Vector3)buoyancyForce, part.transform.position, ForceMode.Force);
        }

        private double CalculateDepth(Vector3d position)
        {
            //We assume the ocean is at 0 altitude
            double depth = -FlightGlobals.getAltitudeAtPos(position, vessel.mainBody);
            return depth;
        }

        private void UpdateWaterContact(double depth)
        {
            if (depth < 0)      //corresponds to above the ocean
            {
                if (part.WaterContact)
                {
                    part.WaterContact = false;
                    part.rigidbody.drag = 0;
                    vessel.checkSplashed();
                }
                return;
            }

            if (!part.WaterContact)
            {
                part.WaterContact = true;
                vessel.checkSplashed();
            }
        }

        private Vector3d BuoyancyForce(Rigidbody body, double depth)
        {
            ApplySplashEffect(depth, body);

            if (CheckDieOnHighVelocity(body))
                return Vector3.zero;

            double vol;
            double depthForMaxForce;
            if(overrideVol > 0)
            {
                vol = overrideVol;
                depthForMaxForce = overridedepthForMaxForce;
            }
            else if (part.collider)
            {
                Vector3 size = part.collider.bounds.size;
                vol = size.x * size.y * size.z;       //This is highly approximate, but it works
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
            part.rigidbody.angularDrag = part.rigidbody.drag;

            Vector3d gForce = -FlightGlobals.getGeeForceAtPosition(part.transform.position);

            Vector3d buoyancyForce = gForce * depthFactor;
            buoyancyForce *= vol * BBPlanetOceanDensity.EvaluateBodyOceanDensity(vessel.mainBody);

            return buoyancyForce;
        }

        private void ApplySplashEffect(double depth, Rigidbody body)
        {
            if (depth > 1)
                return;

            float mag = body.velocity.magnitude * 0.1f;
            if(mag > 10)
                FXMonger.Splash(part.transform.position, mag);
        }

        private bool CheckDieOnHighVelocity(Rigidbody body)
        {
            Vector3d velVector = body.velocity + Krakensbane.GetFrameVelocityV3f();

            double vertVec = Vector3d.Dot(velVector, vessel.upAxis);
            if (Math.Abs(vertVec) > part.crashTolerance * vertCrashTolFactor)
            {
                GameEvents.onCrashSplashdown.Fire(new EventReport(FlightEvents.SPLASHDOWN_CRASH, part, part.partInfo.title, "", 0, ""));
                part.Die();
                return true;
            }
            double horizVel = (velVector - vertVec * vessel.upAxis).magnitude;
            if (horizVel > part.crashTolerance * horizCrashTolFactor)
            {
                GameEvents.onCrashSplashdown.Fire(new EventReport(FlightEvents.SPLASHDOWN_CRASH, part, part.partInfo.title, "", 0, ""));
                part.Die();
                return true;
            }
            return false;
        }
    }
}
