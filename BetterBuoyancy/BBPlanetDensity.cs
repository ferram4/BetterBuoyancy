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

namespace BetterBuoyancy
{
    static class BBPlanetOceanDensity
    {
        static Dictionary<int, double> bodyOceanDensities;

        public static double EvaluateBodyOceanDensity(CelestialBody body)
        {
            if (bodyOceanDensities == null)
            {
                LoadBodyOceanDensitiesFromConfig();
            }
            return bodyOceanDensities[body.flightGlobalsIndex];
        }

        static void LoadBodyOceanDensitiesFromConfig()
        {
            bodyOceanDensities = new Dictionary<int,double>();
            ConfigNode[] oceanNodes = GameDatabase.Instance.GetConfigNodes("BBOceanDensities");
            double defaultDensity = 1;
            foreach (ConfigNode oceanNode in oceanNodes)
            {
                ConfigNode[] bodyNodes = oceanNode.GetNodes("BodyOceanDensity");
                for (int i = 0; i < bodyNodes.Length; i++)
                {
                    ConfigNode bodyNode = bodyNodes[i];
                    if (bodyNode == null)
                        continue;

                    if(!bodyNode.HasValue("bodyIndex") || !bodyNode.HasValue("bodyOceanDensity"))
                    {
                        Debug.LogError("BodyNode " + i + " error; data incomplete");
                        continue;
                    }
                    int bodyIndex = int.Parse(bodyNode.GetValue("bodyIndex"));
                    double density = double.Parse(bodyNode.GetValue("bodyOceanDensity"));

                    bodyOceanDensities.Add(bodyIndex, density);
                }
                if(oceanNode.HasValue("defaultDensity"))
                    defaultDensity = double.Parse(oceanNode.GetValue("defaultDensity"));
            }
            for(int i = 0; i < FlightGlobals.Bodies.Count; i++)
            {
                if (bodyOceanDensities.ContainsKey(i))
                    continue;

                bodyOceanDensities.Add(i, defaultDensity);
            }
        }
    }
}
