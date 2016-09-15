using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Xna.Framework;
//my referances
using Calypsow_Engine.Utility;
using System.Runtime.Serialization;

namespace Calypsow_Engine
{
    namespace Particles 
    {
        namespace Forces
        {
            public interface ParticleForceGenerator
            {
                void UpdateForce(Particle p, float dt);
            }
            [Serializable]
            public class ParticleGravity : ParticleForceGenerator
            {
                static Vector3 Gravity;
                public ParticleGravity()
                {
                    Gravity = new Vector3(0, -10,0);
                }
                public ParticleGravity(float value)
                {
                    Gravity = new Vector3(0, value,0);
                }

                public void UpdateForce(Particle p, float dt)
                {
                    if (p.InversedMass == 0)
                    {
                        return;
                    }
                    p.AddForceToAccumlator(Gravity * p.GetMass());
                }
            }
            [Serializable]
            public class ParticleDrag : ParticleForceGenerator
            {
                float k1, k2;
                public ParticleDrag()
                {
                    this.k1 = 0.0001f;
                    this.k2 = 0.0001f;
                }
                public ParticleDrag(float k1, float k2)
                {
                    this.k1 = k1;
                    this.k2 = k2;
                }
                public void UpdateForce(Particle p, float dt)
                {
                    Vector3 force = p.GetVelocity();
                    float dragcofficient = force.Length();
                    //Calculate the total coefficient
                    dragcofficient = k1 * dragcofficient + k1 * dragcofficient * dragcofficient;
                    //Calculate the final force and apply it
                    force.Normalize();
                    force = Vector3.Multiply(force, (float)-dragcofficient);
                    p.ForceAccumlator += force;
                }
            }
            [Serializable]
            public class ParticleSpring : ParticleForceGenerator
            {
                float K;
                Particle Other;
                float RestLength;
                public ParticleSpring(Particle other, float k, float l0)
                {
                    this.Other = other;
                    this.K = k;
                    this.RestLength = l0;
                }
                public void UpdateForce(Particle p, float dt)
                {
                    //Calculate the Vector of the spring
                    Vector3 force;
                    force = p.GetPosition();
                    force -= Other.GetPosition();
                    //Calculate the magnitude of the force
                    float magnitude = force.Length();
                    magnitude = Math.Abs(magnitude - RestLength);
                    magnitude *= K;
                    //Calculating the final force and apply it
                    force.Normalize();
                    force = Vector3.Multiply(force, (float)-magnitude);
                    p.ForceAccumlator += force;
                }
            }
            [Serializable]
            public class ParticleAnchoredSpring : ParticleForceGenerator
            {
                Vector3 Anchor;
                float K;
                float RestLength;
                public ParticleAnchoredSpring(Vector3 anchor, float k, float l0)
                {
                    this.Anchor = anchor;
                    this.K = k;
                    this.RestLength = l0;
                }
                public void UpdateForce(Particle p, float dt)
                {
                    //Calculate the vector of the spring
                    Vector3 force = p.GetPosition();
                    force -= Anchor;
                    //Calculate The magnitude of the force
                    float magnitude = force.Length();
                    magnitude = Math.Abs(magnitude - RestLength);
                    magnitude *= K;
                    //Calculate the final force and apply it
                    force.Normalize();
                    force = Vector3.Multiply(force, (float)-magnitude);
                    p.ForceAccumlator += force;

                }
            }
            [Serializable]
            public class ParticleEngineForce : ParticleForceGenerator
            {
                Vector3 force;
                public ParticleEngineForce(float x, float y)
                {
                    force = new Vector3(x, y,0);
                }
                public void UpdateForce(Particle p, float dt)
                {
                    p.ForceAccumlator += force;

                }
            }
            [Serializable]
            public class ParticleForceRegistry
            {
                struct ParticleForceRegistration
                {
                    public Particle p;
                    public ParticleForceGenerator pfg;
                    public ParticleForceRegistration(Particle PP, ParticleForceGenerator PFG)
                    {
                        p = PP;
                        pfg = PFG;
                    }
                }
                List<ParticleForceRegistration> Registrations = new List<ParticleForceRegistration>();
                public void Add(Particle p, ParticleForceGenerator pfg)
                {
                    ParticleForceRegistration temp = new ParticleForceRegistration(p, pfg);
                    Registrations.Add(temp);
                }
                public void Remove(Particle p, ParticleForceGenerator pfg)
                {
                    ParticleForceRegistration temp = new ParticleForceRegistration(p, pfg);
                    Registrations.Remove(temp);
                }
                public void Clear()
                {
                    Registrations.Clear();
                }
                public void UpdateForces(float dt)
                {
                    foreach (ParticleForceRegistration item in Registrations)
                    {
                        item.pfg.UpdateForce(item.p, dt);
                    }
                }
            }
        }
    }
    namespace Rigid 
    {
    namespace Forces
    {
        public interface RigidForceGenerator
        {
            void UpdateForce(RigidBody rg, float dt);
        }
        [Serializable]
        public class RigidBodyGravity : RigidForceGenerator
        {
            public static Vector3 Gravity;
            public RigidBodyGravity()
            {
                //XNAAAAAAAAAAA down is positive
                Gravity = new Vector3(0, 10,0);
            }
            public RigidBodyGravity(float value)
            {
                Gravity = new Vector3(0,value,0);
            }
            public void UpdateForce(RigidBody rg, float dt)
            {
                if (rg.GetInverseMass() == 0 || float.IsInfinity(rg.GetMass())) 
                {
                    return;
                }

                rg.ForceAccumlator += rg.GetMass() * Gravity;
            }
        }
        [Serializable]
        public class RigidSpring : RigidForceGenerator
        {
            /**
             * The point of connection of the spring, in local
             * coordinates.
             */
            Vector3 connectionPoint;

            /**
             * The point of connection of the spring to the other object,
             * in that object's local coordinates.
             */
            Vector3 otherConnectionPoint;

            /** The particle at the other end of the spring. */
            RigidBody other;

            /** Holds the sprint constant. */
            float springConstant;

            /** Holds the rest length of the spring. */
            float restLength;
            /** Creates a new spring with the given parameters. */
            public RigidSpring(Vector3 localConnectionPt, RigidBody other, Vector3 otherConnectionPt, float springConstant, float restLength)
            {
                this.springConstant = springConstant;
                this.restLength = restLength;
                this.otherConnectionPoint = otherConnectionPt;
                this.connectionPoint = localConnectionPt;
                this.other = other;
            }

            /** Applies the spring force to the given rigid body. */
            public void UpdateForce(RigidBody rg, float dt)
            {
                if (rg.GetInverseMass() == 0)
                {
                    return;
                }
                // Calculate the two ends in world space

                Vector3 lws = rg.GetPosition() + Vector3.Multiply(rg.xAxis, this.connectionPoint.X) + Vector3.Multiply(rg.yAxis, this.connectionPoint.Y);
                Vector3 ows = other.GetPosition() + Vector3.Multiply(other.xAxis, this.otherConnectionPoint.X) + Vector3.Multiply(other.yAxis, this.otherConnectionPoint.Y);

                // Calculate the vector of the spring
                Vector3 force = lws - ows;

                // Calculate the magnitude of the force
                float magnitude = force.Length();
                magnitude = (magnitude - restLength);
                magnitude *= springConstant;

                // Calculate the final force and apply it
                force.Normalize();
                force = Vector3.Multiply(force, -magnitude);
                rg.AddForce(force);
                rg.AddTorque(force, lws - rg.GetPosition());
            }
        }
        [Serializable]
        public class RigidExplosion : RigidForceGenerator
        {
            //how long the operation time
            public double timePassed;
            //the location
            public Vector3 detonation;


            //imploision
            public double implosionMaxRadius;
            public double implosionMinRadius;
            public double implosionDuration;
            public float implosionForce;

            //wave
            public double waveSpeed;
            public double waveAcceleration;
            public double waveAndImplosionDuration;
            public double widthOfInerval;
            public double widthOfWave;
            public double PeakBlastForce; //peak blast force
            public double fb;

            //body
            //relative to the body!!
            public Vector3 randomPointOfApplication;

            public RigidExplosion(Vector3 d, double iminr, double imaxr, double iduration, float iforce, double wspeed, double wduration, double wwidth, double fb, double waveAcceleration, RigidBody rg)
            {
                this.fb = fb;
                this.waveAcceleration = waveAcceleration;
                this.timePassed = 0;
                this.detonation = d;
                implosionMaxRadius = imaxr;
                implosionMinRadius = iminr;
                implosionDuration = iduration;
                implosionForce = iforce;
                waveSpeed = wspeed;
                waveAndImplosionDuration = wduration + iduration;
                widthOfWave = wwidth;
                this.PeakBlastForce = fb;
                this.widthOfInerval = wspeed * 16;
                randomPointOfApplication = rg.getRandomPoint();
            }

            public void UpdateForce(RigidBody rg, float dt)
            {
                if (rg.GetMass() == 0 || float.IsInfinity(rg.GetMass()))
                {
                    return;
                }

                float xdif = rg.GetPosition().X - detonation.X;
                float ydif = rg.GetPosition().Y - detonation.Y;
                double distance = Math.Sqrt(xdif * xdif + ydif * ydif);

                if (timePassed < implosionDuration)
                {
                    if (distance > implosionMinRadius && distance < implosionMaxRadius)
                    {
                        Vector3 vectorfa = new Vector3(-xdif, -ydif, 0);
                        vectorfa.Normalize();
                        vectorfa.X *= implosionForce;
                        vectorfa.Y *= implosionForce;
                        rg.AddForce(vectorfa);
                    }
                }

                else if (timePassed < waveAndImplosionDuration)
                {
                    this.widthOfInerval = waveSpeed * dt;
                    double waveDuration = timePassed - implosionDuration;
                    float fa;
                    Vector3 vectorfa;
                    if ((waveSpeed * waveDuration) - (widthOfWave * widthOfInerval) <= distance && distance < waveSpeed * waveDuration)
                    {
                        fa = (float)(PeakBlastForce * (1 - (waveSpeed * waveDuration - distance) / widthOfInerval * widthOfWave));
                        if (fa < 0)
                            return;
                        vectorfa = new Vector3(xdif, ydif, 0);
                        vectorfa.Normalize();
                        vectorfa.X *= fa;
                        vectorfa.Y *= fa;
                        rg.AddForce(vectorfa);
                        rg.AddTorque(vectorfa, randomPointOfApplication);

                        //reset
                        randomPointOfApplication = Vector3.Zero;
                    }
                    else if ((waveSpeed * waveDuration) <= distance && distance < (waveSpeed * waveDuration) + widthOfInerval)
                    {
                        fa = (float)PeakBlastForce;
                        if (fa < 0)
                        {
                            return;
                        }
                        vectorfa = new Vector3(xdif, ydif, 0);
                        vectorfa.Normalize();
                        vectorfa.X *= fa;
                        vectorfa.Y *= fa;
                        rg.AddForce(vectorfa);
                        rg.AddTorque(vectorfa, randomPointOfApplication);

                        //reset
                        randomPointOfApplication = Vector3.Zero;
                    }
                    else if (waveSpeed * waveDuration + widthOfInerval <= distance && distance < waveSpeed * waveDuration + (widthOfWave + 1) * widthOfInerval)
                    {
                        fa = (float)(PeakBlastForce * (distance - waveSpeed * waveDuration - widthOfInerval) / widthOfInerval * widthOfWave);
                        if (fa < 0)
                        {
                            return;
                        }
                        vectorfa = new Vector3(xdif, ydif, 0);
                        vectorfa.Normalize();
                        vectorfa.X *= fa;
                        vectorfa.Y *= fa;
                        rg.AddForce(vectorfa);
                        rg.AddTorque(vectorfa, randomPointOfApplication);

                        //reset
                        randomPointOfApplication = Vector3.Zero;
                    }
                    waveSpeed += dt * this.waveAcceleration;
                    PeakBlastForce = fb;
                    PeakBlastForce -= this.fb * (waveDuration / (waveAndImplosionDuration - implosionDuration));
                }
                timePassed += dt;

            }
        }
        [Serializable]
        public class RandomRigidTorque : RigidForceGenerator    
        {
            Vector3 force;
            Vector3 relativelocation;
            public RandomRigidTorque(Vector3 f, Vector3 rl)
            {
                relativelocation = rl;
                force = f;
            }
            public void UpdateForce(RigidBody rg, float dt)
            {
                if (rg.GetInverseMass() ==0 || float.IsInfinity(rg.GetMass()))
                {
                    return;
                }
                rg.AddTorque(force, relativelocation);
            }
        }
        [Serializable]
        public class RandomRigidForce : RigidForceGenerator
        {
            Vector3 force;
            Vector3 relativelocation;
            public RandomRigidForce(Vector3 f, Vector3 rl)
            {
                relativelocation = rl;
                force = f;
            }
            public void UpdateForce(RigidBody rg, float dt)
            {
                if (rg.GetInverseMass() ==0 || float.IsInfinity(rg.GetMass()))
                {
                    return;
                }
                rg.AddForce(force);
                rg.AddTorque(force, relativelocation);
            }
        }
        [Serializable]
        public class ForceRegistry : ISerializable
        {
            [Serializable]
            struct RigidForceRegistration : ISerializable
            {
                public RigidBody rg;
                public RigidForceGenerator rfg;
                public RigidForceRegistration(RigidBody RG, RigidForceGenerator RFG)
                {
                    rg = RG;
                    rfg = RFG;
                }
                public void GetObjectData(SerializationInfo info, StreamingContext context)
                {
                    info.AddValue("rg", rg);
                    info.AddValue("rfg", rfg);
                }
                public RigidForceRegistration(SerializationInfo info, StreamingContext context)
                {
                    rg = (RigidBody)info.GetValue("rg", typeof(RigidBody));
                    rfg = (RigidForceGenerator)info.GetValue("rfg", typeof(RigidForceGenerator));
                }
            }
            List<RigidForceRegistration> Registrations = new List<RigidForceRegistration>();
            public void ClearRegistrationsAccumelators()
            {
                foreach (RigidForceRegistration item in Registrations)
                {
                    item.rg.ForceAccumlator = Vector3.Zero;
                    item.rg.TorqueAccumlator = 0;
                }
            }
            public void Add(RigidBody p, RigidForceGenerator pfg)
            {
                RigidForceRegistration temp = new RigidForceRegistration(p, pfg);
                Registrations.Add(temp);
            }
            public void Remove(RigidBody p, RigidForceGenerator pfg)
            {
                RigidForceRegistration temp = new RigidForceRegistration(p, pfg);
                Registrations.Remove(temp);
            }            public void Clear()
            {
                Registrations.Clear();
            }
            public void UpdateForces(float dt)
            {
                foreach (RigidForceRegistration item in Registrations)
                {
                    item.rfg.UpdateForce(item.rg, dt);
                }
            }
            public void GetObjectData(SerializationInfo info, StreamingContext context)
            {
                info.AddValue("r", Registrations);
            }
            public ForceRegistry(SerializationInfo info, StreamingContext context)
            {
                Registrations = (List<RigidForceRegistration>)info.GetValue("r", typeof(List<RigidForceRegistration>));
            }
            public ForceRegistry()
            {

            }
        }
    }
    }
}
