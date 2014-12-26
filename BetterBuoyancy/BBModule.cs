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

            part.rigidbody.AddForceAtPosition(BuoyancyForce(part.rigidbody), part.transform.position);
        }

        private Vector3d BuoyancyForce(Rigidbody body)
        {
            //We assume the ocean is at 0 altitude
            double depth = -FlightGlobals.getAltitudeAtPos((Vector3d)part.transform.position, vessel.mainBody);
            if (depth < 0)
            {
                if(part.Splashed)
                {
                    part.WaterContact = false;
                    vessel.Splashed = false;
                    part.rigidbody.drag = 0;
                }
                return Vector3.zero;
            }

            if(!part.Splashed)
            {
                ApplySplashEffect();
                part.WaterContact = true;
                vessel.Splashed = true;
                part.rigidbody.drag = 3;
            }
            if (CheckDieOnHighVelocity(body))
                return Vector3.zero;


            double vol;
            double depthForMaxForce;
            if (part.collider)
            {
                Vector3 size = part.collider.bounds.size;
                vol = size.magnitude;       //This is highly approximate, but it works
                depthForMaxForce = Math.Max(size.x, Math.Max(size.y, size.z));      //This is a very, very rough model of partial immersion
            }
            else
            {
                vol = 1;
                depthForMaxForce = 1;
            }

            
            Vector3d buoyancyForce = -FlightGlobals.getGeeForceAtPosition(part.transform.position) * Math.Min(depth / depthForMaxForce, 1);
            buoyancyForce *= vol * part.rb.mass;
            return buoyancyForce;
        }

        private void ApplySplashEffect()
        {
            float mag = rigidbody.velocity.magnitude * 0.1f;
            if(mag > 20)
                FXMonger.Splash(part.transform.position, mag);
        }

        private bool CheckDieOnHighVelocity(Rigidbody body)
        {
            Vector3d velVector = body.velocity + Krakensbane.GetFrameVelocityV3f();

            double vertVec = Vector3d.Dot(velVector, vessel.upAxis);
            if (vertVec > part.crashTolerance * VERT_CRASH_TOL_FACTOR)
            {
                GameEvents.onCrashSplashdown.Fire(new EventReport(FlightEvents.SPLASHDOWN_CRASH, part, part.partInfo.title, "blah", 0, ""));
                part.Die();
                return true;
            }
            double horizVel = (velVector - vertVec * vessel.upAxis).magnitude;
            if (horizVel > part.crashTolerance * HORIZ_CRASH_TOL_FACTOR)
            {
                GameEvents.onCrashSplashdown.Fire(new EventReport(FlightEvents.SPLASHDOWN_CRASH, part, part.partInfo.title, "blah", 0, ""));
                part.Die();
                return true;
            }
            return false;
        }
    }
}
