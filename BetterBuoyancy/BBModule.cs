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

        static System.Random rand = new System.Random();

        double vertCrashTolFactor = 1.2;
        double horizCrashTolFactor = 7;
        double overrideVol = -1;
        double overridedepthForMaxForce = -1;
        double depth = -1;

        double volume = 1;
        double depthFactor = 0;

        float splashDrag = 0;
        int splashFrameDelay = 0;

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
            if (!FlightGlobals.ready || !vessel.mainBody.ocean || part.PhysicsSignificance == (int)Part.PhysicalSignificance.NONE || part.rb == null)
                return;

            if (!moduleInitialized)
            {
                if (part.partBuoyancy == null)
                    return;
                GameObject.Destroy(part.partBuoyancy);
                moduleInitialized = true;
            }

            depth = CalculateDepth((Vector3d)part.transform.position);
            CalcDepthFactor(depth);
            UpdateWaterContact(depth);
            if (depth >= 0)
            {
                ApplyBouyancyForce(BuoyancyForce(part.rb, depth)+ BuoyantDragForce(part.rb, depthFactor));
            }
        }

        private void Update()
        {
            if (splashFrameDelay <= 0)
            {
                if (depth >= 0)
                {
                    ApplySplashEffect(depth, part.Rigidbody);
                    splashFrameDelay = rand.Next(1, 5);
                }
            }
            else
                --splashFrameDelay;
            
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
                    vessel.checkSplashed();
                }
                if(depthFactor < -1)
                {
                    splashDrag = 0;
                    part.rb.drag = 0;
                    part.rb.angularDrag = 0;
                }
                return;
            }

            float tmp = 0.15f;      //base drag

            tmp /= part.Rigidbody.mass;
            if (splashDrag > tmp)
                splashDrag = splashDrag * 0.98f;
            else
                splashDrag = tmp;

            if (!part.WaterContact)
            {
                part.WaterContact = true;
                vessel.checkSplashed();

                float vertVec = Vector3.Dot(part.rb.velocity + Krakensbane.GetFrameVelocityV3f(), vessel.upAxis);
                vertVec *= 0.1f;
                if (vertVec > 1)
                    vertVec = 1;

                splashDrag = Math.Max(20f * vertVec / part.rb.mass, tmp);
            }
            part.rb.drag = splashDrag;
            part.rb.angularDrag = splashDrag * 10;
        }

        private Vector3d BuoyancyForce(Rigidbody body, double depth)
        {

            if (CheckDieOnHighVelocity(body))
                return Vector3.zero;



            Vector3d gForce = -FlightGlobals.getGeeForceAtPosition(part.transform.position);

            Vector3d buoyancyForce = gForce * depthFactor;
            buoyancyForce *= volume * BBPlanetOceanDensity.EvaluateBodyOceanDensity(vessel.mainBody);

            return buoyancyForce;
        }

        private Vector3d BuoyantDragForce(Rigidbody body, double depthFactor)
        {
            float tmp = 0;
            if (depthFactor < 0.5)
                tmp += (float)depthFactor * 2f;     //drag from depth     
            else
                tmp += (float)(1f - depthFactor) * 2f;


            Vector3d dragForce = Vector3.Project(body.velocity, vessel.upAxis);
            double vertVel = dragForce.magnitude;
            dragForce *= vertVel;
            dragForce *= tmp * BBPlanetOceanDensity.EvaluateBodyOceanDensity(vessel.mainBody);

            if (dragForce.magnitude * TimeWarp.fixedDeltaTime / body.mass > vertVel)
                dragForce = vessel.upAxis * vertVel;

            return -dragForce;
        }

        private void CalcDepthFactor(double depth)
        {
            if (overrideVol > 0)
            {
                volume = overrideVol;
                depthFactor = overridedepthForMaxForce;
            }
            else if (part.collider)
            {
                Vector3 size = part.collider.bounds.size;
                volume = size.x * size.y * size.z;       //This is highly approximate, but it works
                depthFactor = Math.Max(Math.Max(size.x, Math.Max(size.y, size.z)), 2);      //This is a very, very rough model of partial immersion
            }
            else
            {
                volume = 1;
                depthFactor = 2;
            }

            depthFactor = depth / depthFactor;

            depthFactor = Math.Min(depthFactor, 1);
        }

        private void ApplySplashEffect(double depth, Rigidbody body)
        {
            if (depth > 1)
                return;

            float mag = body.velocity.magnitude * 0.02f;
            if(mag > 0.05)
                FXMonger.Splash(part.transform.position, mag);
        }

        private bool CheckDieOnHighVelocity(Rigidbody body)
        {
            Vector3d velVector = body.velocity + Krakensbane.GetFrameVelocityV3f();

            double vertVec = Vector3d.Dot(velVector, vessel.upAxis);
            if (Math.Abs(vertVec) > part.crashTolerance * vertCrashTolFactor)
            {
                GameEvents.onCrashSplashdown.Fire(new EventReport(FlightEvents.SPLASHDOWN_CRASH, part, part.partInfo.title, "", 0, ""));
                FXMonger.Splash(part.transform.position, 10);
                part.Die();
                return true;
            }
            double horizVel = (velVector - vertVec * vessel.upAxis).magnitude;
            if (horizVel > part.crashTolerance * horizCrashTolFactor)
            {
                GameEvents.onCrashSplashdown.Fire(new EventReport(FlightEvents.SPLASHDOWN_CRASH, part, part.partInfo.title, "", 0, ""));
                FXMonger.Splash(part.transform.position, 10);
                part.Die();
                return true;
            }
            return false;
        }
    }
}
