using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Xna.Framework;

using Calypsow_Engine.Particles;
using Calypsow_Engine.Utility;
using Calypsow_Engine.Rigid;
using Calypsow_Engine.Rigid.CollisionDetection_Fine;
using Calypsow_Engine.Particles.CollisionResolution;

namespace Calypsow_Engine
{
    namespace Particles 
    {
        namespace CollisionResolution
        {
            public class ParticleContact
            {
                public Particle[] particles = new Particle[2];
                public Vector3 contactNormal;
                public float restitution;
                public float penetration;
                public ParticleContact()
                {

                }
                public ParticleContact(Particle p1, Particle p2, float rest, float pen)
                {
                    contactNormal = (p1.Position - p2.Position);
                    contactNormal.Normalize();
                    restitution = rest;
                    particles[0] = p1;
                    particles[1] = p2;
                    penetration = pen;
                }
                public ParticleContact(Particle p1, Particle p2, float rest, float pen,Vector3 normal)
                {
                    contactNormal = normal;
                    restitution = rest;
                    particles[0] = p1;
                    particles[1] = p2;
                    penetration = pen;
                }
                public void Resolve(float duration)
                {
                    ResolveVelocity(duration);
                    ResolveInterPenetration(duration);
                }
                public float CalculateSeparatingVelocity()
                {
                    Vector3 relativeVelocity = particles[0].GetVelocity();
                    if (particles[1] != null)
                        relativeVelocity -= particles[1].GetVelocity();
                    return Vector3.Dot(relativeVelocity , contactNormal);
                }
                private void ResolveVelocity(float duration)
                {
                    float separatingVelocity = CalculateSeparatingVelocity();
                    if (separatingVelocity > 0)
                    {
                        return;
                    }
                    float newSepVelocity = -separatingVelocity * restitution;

                    //check the velocity build-up due to acceleration only
                    Vector3 accCausedVelocity = particles[0].GetAcceleration();
                    if (particles[1] != null)
                    {
                        accCausedVelocity -= particles[1].GetAcceleration();
                    }
                    //if we'v got a closing velocity due to acceleration build-up
                    //remove it from the new seperating velocity
                    float accCausedSepVelocity = Vector3.Dot(accCausedVelocity, contactNormal) * duration;
                    if (accCausedSepVelocity < 0)
                        newSepVelocity += restitution * accCausedSepVelocity;
                    //make sure we haven't removed more than was there to remove
                    if (newSepVelocity < 0)
                        newSepVelocity = 0;

                    float deltaVelocity = newSepVelocity - separatingVelocity;
                    float totalInverseMass = particles[0].GetInversedMass();
                    if (particles[1] != null)
                        totalInverseMass += particles[1].GetInversedMass();
                    if (totalInverseMass <= 0)
                        return;
                    float impulse = deltaVelocity / totalInverseMass;
                    Vector3 impulsePerIMass = Vector3.Multiply(contactNormal , impulse);
                    particles[0].Velocity = particles[0].GetVelocity() + impulsePerIMass * particles[0].GetInversedMass();
                    if (particles[1] != null)
                    {
                        particles[1].Velocity = particles[1].GetVelocity() + impulsePerIMass * -particles[1].GetInversedMass();
                    }
                }
                private void ResolveInterPenetration(double duration)
                {
                    if (penetration <= 0)
                    {
                        return;
                    }
                    double totalInverseMass = (1.0f / particles[0].GetMass());
                    if (particles[1] != null)
                        totalInverseMass += (1.0f / particles[1].GetMass());
                    if (totalInverseMass <= 0)
                    {
                        return;
                    }
                    Vector3 movePerIMass = Vector3.Multiply(contactNormal , (float)(-penetration / totalInverseMass));
                    particles[0].Position = particles[0].Position + movePerIMass * particles[0].GetInversedMass();
                    if (particles[1] != null)
                    {
                        particles[1].Position = particles[1].Position + movePerIMass * particles[1].GetInversedMass();
                    }
                }
            }
            public class ParticleContactResolver
            {
                public int iterations;
                public int iterationsUsed;
                public ParticleContactResolver(int iterations)
                {
                    this.iterations = iterations;
                    this.iterationsUsed = 0;
                }
                public void SetIterations(int iterations)
                {
                    this.iterations = iterations;
                }
                public void ResolveContacts(List<ParticleContact> contactArray, int numOfContacts, float duration)
                {
                    iterationsUsed = 0;
                    while (iterationsUsed < iterations)
                    {
                        //double max = 0;
                        //int maxIndex = numOfContacts-1;
                        //for (int i = 0; i < numOfContacts; i++)
                        //{
                        //    double pen = contactArray[i].CalculateSeparatingVelocity();
                        //    if (pen < max)
                        //    {
                        //        max = pen;
                        //        maxIndex = i;
                        //    }
                        //}
                        contactArray[iterationsUsed].Resolve(duration);
                        iterationsUsed++;
                    }
                }
            }
            public interface ParticleContactGenerator
            {
                int AddContact(ParticleContact contact, int limit);
            }
            public class Rod : ParticleContactGenerator
            {
                Particle[] p = new Particle[2];
                public Rod(Particle p1,Particle p2,float l)
                {
                    p[0] = p1;
                    p[1] = p2;
                    length = l;
                }
                
                public float length;
                public float currrentLength()
                {
                    return (p[0].Position - p[1].Position).Length();
                }
                public int AddContact(ref ParticleContact contact, int limit)
                {
                    float currlength = currrentLength();
                    if (currlength==length)
                    {
                        return 0;
                    }
                    contact.particles[0] = p[0];
                    contact.particles[1] = p[1];
                    Vector3 normal = p[1].Position - p[0].Position;
                    normal.Normalize();
                    if (currlength > length) 
                    {
                        contact.contactNormal = normal;
                        contact.penetration = currlength - length;
                    }
                    else
                    {
                        contact.contactNormal = normal * -1;
                        contact.penetration = length - currlength;
                    }
                    contact.restitution = 0;
                    return 0;
                }

                public int AddContact(ParticleContact contact, int limit)
                {
                    return 1;
                }
            }
        }
    }
    namespace Rigid 
    {        
        namespace CollisionResolution
        {
            [Serializable]
            public class FixedJoint
            {
                /**
                 * Holds the two rigid bodies that are connected by this joint.
                 */
                public RigidBody body;
                /**
                 * Holds the relative location of the connection for each
                 * body, given in local coordinates.
                 */
                public Vector3 Position;
                /**
                 * Holds the maximum displacement at the joint before the
                 * joint is considered to be violated. This is normally a
                 * small, epsilon value.  It can be larger, however, in which
                 * case the joint will behave as if an inelastic cable joined
                 * the bodies at their joint locations.
                 */
                public float Error;
                /**
                 * Configures the joint in one go.
                 */
       
                /**
                 * Generates the contacts required to restore the joint if it
                 * has been violated.
                 */
                public void setPosition(int x, int y)
                {
                    this.Position.X = x;
                    this.Position.Y = y;
                }
                public FixedJoint(RigidBody a,Vector3 p,float err)
                {
                    this.body = a;
                    this.Position = p;
                    this.Error = err;
                }
                public void AddContact(ref CollisionData data)
                {
                    // Calculate the length of the joint
                    
                    Vector3 a_to_b = body.Position - Position;
                    Vector3 normal = a_to_b;
                    float length = a_to_b.Length();

                    float penetration = length - Error;

                    // Check if it is violated
                    if (penetration > 0)
                    {
                        normal.Normalize();
                        normal =  -1 * normal;
                        Contact c = new Contact();
                        c.ContactNormal = normal;
                        c.Friction = 0;
                        c.Restitution = 0.5f;
                        c.ContactPoint = (body.Position +this.Position) * 0.5f;
                        c.Penetration = penetration;
                        c.setBodyData(body, null, 1.0f, 0);
                        c.contactToWorld.M11 = normal.X;
                        c.contactToWorld.M12 = -normal.Y;
                        c.contactToWorld.M21 = normal.Y;
                        c.contactToWorld.M22 = normal.X;
                        data.Contacts.Add(c);
                        data.AddContacts(1);
                    }
                    return;
                }
            }
            [Serializable]
            public class Joint
            {
                public bool breakable = false;
                /**
                 * Holds the two rigid bodies that are connected by this joint.
                 */
                public RigidBody[] bodies;
                /**
                 * Holds the relative location of the connection for each
                 * body, given in local coordinates.
                 */
                public Vector3[] Position;
                /**
                 * Holds the maximum displacement at the joint before the
                 * joint is considered to be violated. This is normally a
                 * small, epsilon value.  It can be larger, however, in which
                 * case the joint will behave as if an inelastic cable joined
                 * the bodies at their joint locations.
                 */
                public float Error;
                /**
                 * Configures the joint in one go.
                 */
                public Joint(Joint j)
                {
                    this.Set(j.bodies[0], j.Position[0], j.bodies[1], j.Position[1], j.Error);
                }
                public Joint(RigidBody a, Vector3 a_pos, RigidBody b, Vector3 b_pos, float error)
                {
                    this.Set(a, a_pos, b, b_pos, error);
                }
                public void Set(RigidBody a, Vector3 a_pos, RigidBody b, Vector3 b_pos, float error)
                {
                    bodies = new RigidBody[2];
                    Position = new Vector3[2];
                    bodies[0] = a;
                    bodies[1] = b;
                    Position[0] = a_pos;
                    Position[1] = b_pos;
                    this.Error = error;
                    if (bodies[0] is Box && bodies[1] is Circle)
                    {
                        if (!((Box)bodies[0]).JointedCircles.Contains(bodies[1] as Circle))
                        {
                            ((Box)bodies[0]).JointedCircles.Add(bodies[1] as Circle);
                        }
                    }
                    if (bodies[1] is Box && bodies[0] is Circle)
                    {
                        if (!((Box)bodies[1]).JointedCircles.Contains(bodies[0] as Circle))
                        {
                            ((Box)bodies[1]).JointedCircles.Add(bodies[0] as Circle);
                        }
                    }
                }
                /**
                 * Generates the contacts required to restore the joint if it
                 * has been violated.
                 */
                public bool AddContact(ref CollisionData data)
                {
                    // Calculate the length of the joint
                    Vector3 secondPoint = this.bodies[1].GetPosition() + Vector3.Multiply(this.bodies[1].xAxis, this.Position[1].X) + Vector3.Multiply(this.bodies[1].yAxis, this.Position[1].Y);
                    Vector3 firstPoint = this.bodies[0].GetPosition() + Vector3.Multiply(this.bodies[0].xAxis, this.Position[0].X) + Vector3.Multiply(this.bodies[0].yAxis, this.Position[0].Y);

                    Vector3 a_to_b = secondPoint - firstPoint;
                    Vector3 normal = a_to_b;
                    float length = a_to_b.Length();

                    float penetration = length - Error;

                    //Check if it is violated to break or to add contact
                    if (breakable && Math.Abs(penetration) > 15)
                    {
                        return false;
                    }
                    else
                        if (Math.Abs(penetration) > 0.078)
                        {
                            Console.WriteLine(penetration);
                            normal.Normalize();
                            normal = (penetration > 0 ? normal : -1 * normal);
                            Contact c = new Contact();
                            c.ContactNormal = normal;
                            c.Friction = 1.0f;
                            c.Restitution = 1.0f;
                            c.ContactPoint = (firstPoint + secondPoint) * 0.5f;
                            c.Penetration = Math.Abs(penetration);
                            c.setBodyData(bodies[0], bodies[1], 1.0f, 0);
                            c.contactToWorld.M11 = normal.X;
                            c.contactToWorld.M12 = -normal.Y;
                            c.contactToWorld.M21 = normal.Y;
                            c.contactToWorld.M22 = normal.X;
                            bodies[0].SetCanSleep(true);
                            bodies[1].SetCanSleep(true);
                            data.Contacts.Add(c);
                            data.AddContacts(1);
                            return true;
                        }
                    return true;
                }
                //public bool AddContact(ref CollisionData data)
                //{
                //    Particle p1 = new Particle(bodies[0].GetMass(), bodies[0].Position, bodies[0].Velocity, bodies[0].Acceleration);
                //    Particle p2 = new Particle(bodies[1].GetMass(), bodies[1].Position, bodies[1].Velocity, bodies[1].Acceleration);
                //    Rod r = new Rod(p1, p2, bodies[0].GetDiameter() + bodies[1].GetDiameter() + 2);
                //    ParticleContact pc=new ParticleContact();
                //    r.AddContact(ref pc, 1);
                //    ParticleContactResolver pcr = new ParticleContactResolver(1);
                //    pcr.ResolveContacts(new List<ParticleContact>() { pc }, 1, 0.2f);
                //    bodies[0].Position = p1.Position;
                //    bodies[1].Position = p2.Position;
                //    return true;
                //}
                            
                public bool IsPartOf(RigidBody rg)
                {
                    return (rg == bodies[0] || rg == bodies[1]);
                }

                public static float getError(Box a, Vector3 a_pos, Box b, Vector3 b_pos)
                {
                    return 1.0f;
                }
                public static float getError(Box a, Vector3 a_pos, Circle b, Vector3 b_pos)
                {
                    if (a_pos == Vector3.Zero)
                    {
                        return 1.0f + a.GetDiameter() + b.GetDiameter();
                    }
                    if (b_pos == Vector3.Zero)
                    {
                        return 1.0f + b.GetDiameter();
                    }
                    else
                    {
                        return 1.0f + b.GetDiameter() - b_pos.Length();
                    }
                }
                public static bool getPointsOfApplication(Box a, Box b, ref Vector3[] a_pos, ref Vector3[] b_pos, int state)
                {
                    if (state == 2)
                    {
                        return true;
                    }
                    Vector3 firstPosition = a.GetPosition();
                    Vector3 secondPosition = b.GetPosition();
                    Vector3 relativePosition = secondPosition - firstPosition;
                    relativePosition = Matrix2.M_V(relativePosition, -a.Orientation);
                    a_pos = new Vector3[3];
                    if (relativePosition.X >= 0 && Math.Abs(relativePosition.X) >= Math.Abs(relativePosition.Y))
                    {
                        a_pos[0] = new Vector3(a.GetDiameter(), 0, 0);
                        a_pos[1] = new Vector3(a.GetDiameter(), a.GetDiameter(), 0);
                        a_pos[2] = new Vector3(a.GetDiameter(), -a.GetDiameter(), 0);
                        if (a.joints[Box.Direction.RIGHT] == true)
                            return false;
                        a.joints[Box.Direction.RIGHT] = true;
                        a.jointedWith[Box.Direction.RIGHT] = b;
                        //setting new position
                        Vector3 newPosition = new Vector3(a.GetDiameter() + b.GetDiameter() + 3.0f, 0.0f, 0.0f);
                        newPosition = Matrix2.M_V(newPosition, a.Orientation);
                        newPosition += a.GetPosition();
                        b.Position = newPosition;

                        return getPointsOfApplication(b, a, ref b_pos, ref a_pos, state + 1);
                    }
                    else if (relativePosition.X <= 0 && Math.Abs(relativePosition.X) >= Math.Abs(relativePosition.Y))
                    {
                        a_pos[0] = new Vector3(-a.GetDiameter(), 0, 0);
                        a_pos[1] = new Vector3(-a.GetDiameter(), a.GetDiameter(), 0);
                        a_pos[2] = new Vector3(-a.GetDiameter(), -a.GetDiameter(), 0);
                        if (a.joints[Box.Direction.LEFT] == true)
                            return false;
                        a.joints[Box.Direction.LEFT] = true;
                        a.jointedWith[Box.Direction.LEFT] = b;
                        //setting new position
                        Vector3 newPosition = new Vector3(-a.GetDiameter() - b.GetDiameter() - 3.0f, 0.0f, 0.0f);
                        newPosition = Matrix2.M_V(newPosition, a.Orientation);
                        newPosition += a.GetPosition();
                        b.Position = newPosition;

                        return getPointsOfApplication(b, a, ref b_pos, ref a_pos, 1 + state);
                    }
                    else if (relativePosition.Y >= 0 && Math.Abs(relativePosition.X) <= Math.Abs(relativePosition.Y))
                    {
                        a_pos[0] = new Vector3(0, a.GetDiameter(), 0);
                        a_pos[1] = new Vector3(a.GetDiameter(), a.GetDiameter(), 0);
                        a_pos[2] = new Vector3(-a.GetDiameter(), a.GetDiameter(), 0);
                        if (a.joints[Box.Direction.DOWN] == true)
                            return false;
                        a.joints[Box.Direction.DOWN] = true;
                        a.jointedWith[Box.Direction.DOWN] = b;
                        //setting new position
                        Vector3 newPosition = new Vector3(0.0f, a.GetDiameter() + b.GetDiameter() + 3.0f, 0.0f);
                        newPosition = Matrix2.M_V(newPosition, a.Orientation);
                        newPosition += a.GetPosition();
                        b.Position = newPosition;

                        return getPointsOfApplication(b, a, ref b_pos, ref a_pos, 1 + state);
                    }
                    else if (relativePosition.Y <= 0 && Math.Abs(relativePosition.X) <= Math.Abs(relativePosition.Y))
                    {
                        a_pos[0] = new Vector3(0, -a.GetDiameter(), 0);
                        a_pos[1] = new Vector3(a.GetDiameter(), -a.GetDiameter(), 0);
                        a_pos[2] = new Vector3(-a.GetDiameter(), -a.GetDiameter(), 0);
                        if (a.joints[Box.Direction.UP] == true)
                            return false;
                        a.joints[Box.Direction.UP] = true;
                        a.jointedWith[Box.Direction.UP] = b;
                        //setting new position
                        Vector3 newPosition = new Vector3(0.0f, -a.GetDiameter() - b.GetDiameter() - 3.0f, 0.0f);
                        newPosition = Matrix2.M_V(newPosition, a.Orientation);
                        newPosition += a.GetPosition();
                        b.Position = newPosition;

                        return getPointsOfApplication(b, a, ref b_pos, ref a_pos, 1+state);
                    }
                    return false;
                }
                public static bool getPointsofApplication(Box a, Circle b, ref Vector3[] a_pos, ref Vector3[] b_pos)
                {
                    Vector3 firstPosition = a.GetPosition();
                    Vector3 secondPosition = b.GetPosition();
                    b_pos = new Vector3[2];
                    b_pos[0] = Vector3.Zero;
                    b_pos[1] = Vector3.Zero;
                    a_pos = new Vector3[2];
                    a_pos[1] = Vector3.Zero;
                    Vector3 relativePosition = secondPosition - firstPosition;
                    relativePosition = Matrix2.M_V(relativePosition, -a.Orientation);
                    if (relativePosition.X >= 0 && Math.Abs(relativePosition.X) >= Math.Abs(relativePosition.Y))
                    {
                        a_pos[0] = new Vector3(a.GetDiameter(), 0, 0);
                        if (a.joints[Box.Direction.RIGHT] == true)
                            return false;
                        a.joints[Box.Direction.RIGHT] = true;
                        a.jointedWith[Box.Direction.RIGHT] = b;
                        //setting new position
                        Vector3 newPosition = new Vector3(a.GetDiameter() + b.GetDiameter() + 3.0f, 0.0f, 0.0f);
                        newPosition = Matrix2.M_V(newPosition, a.Orientation);
                        newPosition += a.GetPosition();
                        b.Position = newPosition;
                        b.joint = true;
                        b.jointedWith[0] = a;
                    }
                    else if (relativePosition.X <= 0 && Math.Abs(relativePosition.X) >= Math.Abs(relativePosition.Y))
                    {
                        a_pos[0] = new Vector3(-a.GetDiameter(), 0, 0);
                        if (a.joints[Box.Direction.LEFT] == true)
                            return false;
                        a.joints[Box.Direction.LEFT] = true;
                        a.jointedWith[Box.Direction.LEFT] = b;
                        //setting new position
                        Vector3 newPosition = new Vector3(-a.GetDiameter() - b.GetDiameter() - 3.0f, 0.0f, 0.0f);
                        newPosition = Matrix2.M_V(newPosition, a.Orientation);
                        newPosition += a.GetPosition();
                        b.Position = newPosition;
                        b.joint = true;
                        b.jointedWith[0] = a;
                    }
                    else if (relativePosition.Y >= 0 && Math.Abs(relativePosition.X) <= Math.Abs(relativePosition.Y))
                    {
                        a_pos[0] = new Vector3(0, a.GetDiameter(), 0);
                        if (a.joints[Box.Direction.DOWN] == true)
                            return false;
                        a.joints[Box.Direction.DOWN] = true;
                        a.jointedWith[Box.Direction.DOWN] = b;
                        //setting new position
                        Vector3 newPosition = new Vector3(0.0f, a.GetDiameter() + b.GetDiameter() + 3.0f, 0.0f);
                        newPosition = Matrix2.M_V(newPosition, a.Orientation);
                        newPosition += a.GetPosition();
                        b.Position = newPosition;
                        b.joint = true;
                        b.jointedWith[0] = a;
                    }
                    else if (relativePosition.Y <= 0 && Math.Abs(relativePosition.X) <= Math.Abs(relativePosition.Y))
                    {
                        a_pos[0] = new Vector3(0, -a.GetDiameter(), 0);
                        if (a.joints[Box.Direction.UP] == true)
                            return false;
                        a.joints[Box.Direction.UP] = true;
                        a.jointedWith[Box.Direction.UP] = b;

                        //setting new position
                        Vector3 newPosition = new Vector3(0.0f, -a.GetDiameter() - b.GetDiameter() - 3.0f, 0.0f);
                        newPosition = Matrix2.M_V(newPosition, a.Orientation);
                        newPosition += a.GetPosition();
                        b.Position = newPosition;
                        b.joint = true;
                        b.jointedWith[0] = a;
                    }
                    return true;
                }
                //rigidbody
                public static bool getPointsofApplication(RigidBody a, RigidBody b, ref Vector3[] a_pos, ref Vector3[] b_pos)
                {
                    if (a.isBox() && b.isBox())
                    {
                        return getPointsOfApplication((Box)a, (Box)b, ref a_pos, ref b_pos, 0);
                    }
                    else if (a.isBox() && b.isCircle())
                    {
                        return getPointsofApplication((Box)a, (Circle)b, ref a_pos, ref b_pos);
                    }
                    else if (a.isCircle() && b.isBox())
                    {
                        return getPointsofApplication((Box)b, (Circle)a, ref b_pos, ref a_pos);
                    }
                    return false;
                }
                public static float getError(RigidBody a, Vector3 a_pos, RigidBody b, Vector3 b_pos)
                {
                    if (a.isBox() && b.isBox())
                    {
                        return getError((Box)a, a_pos, (Box)b, b_pos);
                    }
                    else if (a.isBox() && b.isCircle())
                    {
                        return getError((Box)a, a_pos, (Circle)b, b_pos);
                    }
                    else if (b.isBox() && a.isCircle())
                    {
                        return getError((Box)b, b_pos, (Circle)a, a_pos);
                    }
                    return 0.0f;
                }
            }
            public class RigidRod 
            {
                public RigidBody[] bodies;
                public Particle[] particles;
                public float maxLength;
                public const float error = 5.0f;
                public ParticleContactResolver pcr = new ParticleContactResolver(2);
                public RigidRod(RigidBody r1,RigidBody r2)
                {
                    bodies = new RigidBody[2];
                    particles = new Particle[2];
                    bodies[0] = r1;
                    bodies[1] = r2;
                    set(r1, r2);
                }
                public void set(RigidBody r1, RigidBody r2) 
                {
                    particles[0] = new Particle(bodies[0].GetMass(), bodies[0].Position, bodies[0].Velocity, bodies[0].Acceleration);
                    particles[1] = new Particle(bodies[1].GetMass(), bodies[1].Position, bodies[1].Velocity, bodies[1].Acceleration);
                    maxLength = r1.GetDiameter()+r2.GetDiameter()+error;
                }
                public float currentLength(Particle p1,Particle p2) 
                {
                    return (p1.Position - p2.Position).Length() + error;
                }
                public void Adjust(float duration) 
                {
                    List<ParticleContact> temp = new List<ParticleContact>();
                    AddContact(particles[0], particles[1], ref temp);
                    AddContact(particles[1], particles[0], ref temp);
                    pcr.iterations = temp.Count;
                    pcr.ResolveContacts(temp, temp.Count, duration);
                    bodies[0].Position = particles[0].Position;
                    bodies[1].Position = particles[1].Position;
                }
                public void AddContact(Particle p1,Particle p2,ref List<ParticleContact> l)
                {
                    float currlength = currentLength(p1 ,p2);
                    if (currlength == maxLength)
                        return;
                    Vector3 normal = p2.Position - p1.Position;
                    if(currlength > maxLength)
                    {
                        normal.Normalize();
                        l.Add(new ParticleContact(p1, p2, 0,currlength - maxLength,normal));
                    }
                    else
                    {
                        normal = normal * -1;
                        normal.Normalize();
                        l.Add(new ParticleContact(p1, p2, 0,maxLength - currlength,normal));
                    }
                }
                public bool IsPartOf(RigidBody rg)
                {
                    return (rg == bodies[0] || rg == bodies[1]);
                }
            }
            [Serializable]
            public class Contact
            {
                public RigidBody[] bodies = new RigidBody[2];
                public float Restitution;
                public Vector3 ContactNormal;
                public float Penetration;
                public Vector3 ContactPoint;
                public float Friction;
                /**
                * A transform matrix that converts coordinates in the contact’s
                * frame of reference to world coordinates. The columns of this
                * matrix form an orthonormal set of vectors.
                */
                public Matrix contactToWorld;
                /**
                * Holds the closing velocity at the point of contact. This is
                * set when the calculateInternals function is run.
                */
                public Vector3 contactVelocity;
                /**
                * Holds the required change in velocity for this contact to be
                * resolved.
                */
                public float desiredDeltaVelocity;
                /**
                * Holds the world space position of the contact point
                * relative to the center of each body. This is set when
                * the calculateInternals function is run.
                */
                public Vector3[] relativeContactPosition;
                public Contact()
                {
                    bodies[0] = null;
                    bodies[1] = null;
                    Restitution = 0.5f;
                    ContactNormal = new Vector3();
                    Penetration = 0;
                    relativeContactPosition = new Vector3[2];
                }
                public void MatchAwakeState()
                {
                    // Collisions with the world never cause a body to wake up.
                    if (bodies[1] == null) return;
                    bool body0awake = bodies[0].GetisAwake();
                    bool body1awake = bodies[1].GetisAwake();

                    // Wake up only the sleeping one
                    if (!(body0awake && body1awake))
                    {
                        if (body0awake) bodies[1].SetAwake(true);
                        else bodies[0].SetAwake(true);
                    }
                }
                public void setBodyData(RigidBody one, RigidBody two, float friction, float restitution)
                {
                    bodies[0] = one;
                    bodies[1] = two;
                    this.Friction = friction;
                    this.Restitution = restitution;
                }
                /*
                 * Swaps the bodies in the current contact, so body 0 is at body 1 and
                 * vice versa. This also changes the direction of the contact normal,
                 * but doesn't update any calculated internal data. If you are calling
                 * this method manually, then call calculateInternals afterwards to
                 * make sure the internal data is up to date.
                 */
                public void SwapBodies()
                {
                    ContactNormal *= -1;
                    RigidBody temp = bodies[0];
                    bodies[0] = bodies[1];
                    bodies[1] = temp;
                }
                public Vector3 CalculateLocalVelocity(int bodyIndex, float duration)
                {
                    RigidBody thisBody = bodies[bodyIndex];

                    // Work out the velocity of the contact point.
                    Vector3 velocity =
                        Vector3.Cross(new Vector3(0, 0, thisBody.GetRotation()), relativeContactPosition[bodyIndex]);
                    velocity += thisBody.GetVelocity();

                    // Turn the velocity into contact-coordinates.
                    Vector3 contactVelocity = Matrix2.transformTranspose(contactToWorld, velocity);

                    // Calculate the ammount of velocity that is due to forces without
                    // reactions.
                    Vector3 accVelocity = thisBody.GetLastFrameAcceleration() * duration;

                    // Calculate the velocity in contact-coordinates.
                    accVelocity = Matrix2.transformTranspose(contactToWorld, accVelocity);

                    // We ignore any component of acceleration in the contact normal
                    // direction, we are only interested in planar acceleration
                    accVelocity.X = 0;

                    // Add the planar velocities - if there's enough friction they will
                    // be removed during velocity resolution
                    contactVelocity += accVelocity;

                    // And return it
                    return contactVelocity;
                }
                public void CalculateDesiredDeltaVelocity(float duration)
                {
                    const float velocityLimit = (float)0.25f;

                    // Calculate the acceleration induced velocity accumulated this frame
                    float velocityFromAcc = 0;

                    if (bodies[0].GetisAwake())
                    {
                        velocityFromAcc = Vector3.Dot(bodies[0].GetLastFrameAcceleration() * duration, ContactNormal);
                    }
                    if (bodies[1] != null && bodies[1].GetisAwake())
                    {
                        velocityFromAcc -=
                            Vector3.Dot(bodies[1].GetLastFrameAcceleration() * duration, ContactNormal);
                    }
                    // If the velocity is very slow, limit the restitution
                    float thisRestitution = Restitution;
                    if (Math.Abs(contactVelocity.X) < velocityLimit)
                    {
                        thisRestitution = (float)0.0f;
                    }

                    // Combine the bounce velocity with the removed
                    // acceleration velocity.
                    desiredDeltaVelocity = -contactVelocity.X - thisRestitution * (contactVelocity.X - velocityFromAcc);
                }
                public void CalculateInternals(float duration)
                {
                    // Check if the first object is NULL, and swap if it is.
                    if (bodies[0] == null) SwapBodies();

                    // Store the relative position of the contact relative to each body

                    relativeContactPosition[0] = ContactPoint - bodies[0].GetPosition();
                    if (bodies[1] != null)
                    {
                        relativeContactPosition[1] = ContactPoint - bodies[1].GetPosition();
                    }

                    // Find the relative velocity of the bodies at the contact point.
                    contactVelocity = CalculateLocalVelocity(0, duration);
                    if (bodies[1] != null)
                    {
                        contactVelocity -= CalculateLocalVelocity(1, duration);
                    }

                    // Calculate the desired change in velocity for resolution
                    CalculateDesiredDeltaVelocity(duration);
                }
                Vector3 CalculateFrictionlessImpulse(Matrix[] inverseInertiaTensor)
                {
                    Vector3 impulseContact;

                    // Build a vector that shows the change in velocity in
                    // world space for a unit impulse in the direction of the contact
                    // normal.
                    Vector3 deltaVelWorld = Vector3.Cross(relativeContactPosition[0], ContactNormal);
                    deltaVelWorld = Matrix2.transform(inverseInertiaTensor[0], deltaVelWorld);
                    deltaVelWorld = Vector3.Cross(deltaVelWorld, relativeContactPosition[0]);

                    // Work out the change in velocity in contact coordiantes.
                    float deltaVelocity = Vector3.Dot(deltaVelWorld, ContactNormal);

                    // Add the linear component of velocity change
                    deltaVelocity += bodies[0].GetInverseMass();

                    // Check if we need to the second body's data
                    if (bodies[1] != null)
                    {
                        // Go through the same transformation sequence again
                        /*vector3*/
                        deltaVelWorld = Vector3.Cross(relativeContactPosition[1], ContactNormal);
                        deltaVelWorld = Matrix2.transform(inverseInertiaTensor[1], deltaVelWorld);
                        deltaVelWorld = Vector3.Cross(deltaVelWorld, relativeContactPosition[1]);

                        // Add the change in velocity due to rotation
                        deltaVelocity += Vector3.Dot(deltaVelWorld, ContactNormal);

                        // Add the change in velocity due to linear motion
                        deltaVelocity += bodies[1].GetInverseMass();
                    }

                    // Calculate the required size of the impulse
                    impulseContact.X = desiredDeltaVelocity / deltaVelocity;
                    impulseContact.Y = 0;
                    impulseContact.Z = 0;
                    return impulseContact;
                }
                Vector3 CalculateFrictionImpulse(Matrix[] inverseInertiaTensor)
                {
                    Vector3 impulseContact;
                    float inverseMass = bodies[0].GetInverseMass();

                    // The equivalent of a cross product in matrices is multiplication
                    // by a skew symmetric matrix - we build the matrix for converting
                    // between linear and angular quantities.
                    Matrix impulseToTorque = Matrix2.setSkewSymmetric(relativeContactPosition[0]);
                    //impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);

                    // Build the matrix to convert contact impulse to change in velocity
                    // in world coordinates.
                    Matrix deltaVelWorld = impulseToTorque;
                    deltaVelWorld *= inverseInertiaTensor[0];
                    deltaVelWorld *= impulseToTorque;
                    deltaVelWorld *= -1;

                    // Check if we need to add body two's data
                    if (bodies[1] != null)
                    {
                        // Set the cross product matrix
                        impulseToTorque = Matrix2.setSkewSymmetric(relativeContactPosition[1]);

                        // Calculate the velocity change matrix
                        Matrix deltaVelWorld2 = impulseToTorque;
                        deltaVelWorld2 *= inverseInertiaTensor[1];
                        deltaVelWorld2 *= impulseToTorque;
                        deltaVelWorld2 *= -1;

                        deltaVelWorld2.M44 = 1;

                        // Add to the total delta velocity.
                        deltaVelWorld += deltaVelWorld2;

                        // Add to the inverse mass
                        inverseMass += bodies[1].GetInverseMass();
                    }

                    // Do a change of basis to convert into contact coordinates.
                    Matrix deltaVelocity = Matrix.Transpose(contactToWorld);
                    deltaVelocity.M44 = 1;
                    deltaVelocity *= deltaVelWorld;
                    deltaVelocity *= contactToWorld;

                    // Add in the linear velocity change
                    deltaVelocity.M11 += inverseMass;
                    deltaVelocity.M22 += inverseMass;
                    deltaVelocity.M33 += inverseMass;
                    deltaVelocity.M44 = 1;

                    // Invert to get the impulse needed per unit velocity
                    Matrix impulseMatrix = Matrix.Invert(deltaVelocity);

                    // Find the target velocities to kill
                    Vector3 velKill = new Vector3(desiredDeltaVelocity,
                        -contactVelocity.Y,
                        -contactVelocity.Z);

                    // Find the impulse to kill target velocities
                    impulseContact = Matrix2.transform(impulseMatrix, velKill);

                    // Check for exceeding friction
                    float planarImpulse = (float)Math.Sqrt(
                        impulseContact.Y * impulseContact.Y +
                        impulseContact.Z * impulseContact.Z
                        );
                    if (planarImpulse > impulseContact.X * Friction)
                    {
                        // We need to use dynamic friction
                        impulseContact.Y /= planarImpulse;
                        impulseContact.Z /= planarImpulse;

                        impulseContact.X = deltaVelocity.M11 +
                            deltaVelocity.M12 * Friction * impulseContact.Y +
                            deltaVelocity.M13 * Friction * impulseContact.Z;
                        impulseContact.X = desiredDeltaVelocity / impulseContact.X;
                        impulseContact.Y *= Friction * impulseContact.X;
                        impulseContact.Z *= Friction * impulseContact.X;
                    }
                    //impulseContact.Z = 0;
                    return impulseContact;
                }
                public void ApplyVelocityChange(Vector3[] velocityChange, Vector3[] rotationChange)
                {
                    // Get hold of the inverse mass and inverse inertia tensor, both in
                    // world coordinates.
                    Matrix[] inverseInertiaTensor = new Matrix[2];
                    inverseInertiaTensor[0] = bodies[0].GetInverseInertiaTensorWorld();
                    if (bodies[1] != null)
                        inverseInertiaTensor[1] = bodies[1].GetInverseInertiaTensorWorld();

                    // We will calculate the impulse for each contact axis
                    Vector3 impulseContact;

                    if (Friction == (float)0.0)
                    {
                        // Use the short format for frictionless contacts
                        impulseContact = CalculateFrictionlessImpulse(inverseInertiaTensor);
                    }
                    else
                    {
                        // Otherwise we may have impulses that aren't in the direction of the
                        // contact, so we need the more complex version.
                        impulseContact = CalculateFrictionImpulse(inverseInertiaTensor);
                    }

                    // Convert impulse to world coordinates
                    Vector3 impulse = Matrix2.transform(contactToWorld, impulseContact);

                    // Split in the impulse into linear and rotational components
                    Vector3 impulsiveTorque = Vector3.Cross(relativeContactPosition[0], impulse);
                    rotationChange[0] = Matrix2.transform(inverseInertiaTensor[0], impulsiveTorque);
                    velocityChange[0].X = 0;
                    velocityChange[0].Y = 0;
                    velocityChange[0].Z = 0;

                    velocityChange[0] += (impulse * bodies[0].GetInverseMass());

                    // Apply the changes
                    bodies[0].SetVelocity(bodies[0].GetVelocity() + velocityChange[0]);
                    bodies[0].SetRotation(bodies[0].GetRotation() + rotationChange[0].Z);

                    if (bodies[1] != null)
                    {
                        // Work out body one's linear and angular changes
                        /*vector3*/
                        impulsiveTorque = Vector3.Cross(impulse, relativeContactPosition[1]);
                        rotationChange[1] = Matrix2.transform(inverseInertiaTensor[1], impulsiveTorque);//inverseInertiaTensor[1].transform(impulsiveTorque);
                        velocityChange[1].X = 0;
                        velocityChange[1].Y = 0;
                        velocityChange[1].Z = 0;
                        velocityChange[1] += (impulse * -bodies[1].GetInverseMass());

                        // And apply them.
                        bodies[1].SetVelocity(bodies[1].GetVelocity() + velocityChange[1]);
                        bodies[1].SetRotation(bodies[1].GetRotation() + rotationChange[1].Z);
                    }
                }
                public void ApplyPositionChange(Vector3[] linearChange, Vector3[] angularChange, float penetration)
                {
                    const float angularLimit = (float)0.2f;
                    float[] angularMove = new float[2];
                    float[] linearMove = new float[2];

                    float totalInertia = 0;
                    float[] linearInertia = new float[2];
                    float[] angularInertia = new float[2];

                    // We need to work out the inertia of each object in the direction
                    // of the contact normal, due to angular inertia only.
                    for (int i = 0; i < 2; i++)
                    {
                        if (bodies[i] != null)
                        {
                            Matrix inverseInertiaTensor;
                            inverseInertiaTensor = bodies[i].GetInverseInertiaTensorWorld();

                            // Use the same procedure as for calculating frictionless
                            // velocity change to work out the angular inertia.
                            Vector3 angularInertiaWorld =
                                Vector3.Cross(relativeContactPosition[i], ContactNormal);
                            angularInertiaWorld =
                                Matrix2.transform(inverseInertiaTensor, angularInertiaWorld);//inverseInertiaTensor.transform(angularInertiaWorld);
                            angularInertiaWorld =
                                Vector3.Cross(angularInertiaWorld, relativeContactPosition[i]);
                            angularInertia[i] =
                                Vector3.Dot(angularInertiaWorld, ContactNormal);

                            // The linear component is simply the inverse mass
                            linearInertia[i] = bodies[i].GetInverseMass();

                            // Keep track of the total inertia from all components
                            totalInertia += linearInertia[i] + angularInertia[i];

                            // We break the loop here so that the totalInertia value is
                            // completely calculated (by both iterations) before
                            // continuing.
                        }
                    }
                    // Loop through again calculating and applying the changes
                    for (int i = 0; i < 2; i++) if (bodies[i] != null)
                        {
                            // The linear and angular movements required are in proportion to
                            // the two inverse inertias.
                            float sign = (i == 0) ? 1 : -1;
                            angularMove[i] =
                                sign * penetration * (angularInertia[i] / totalInertia);
                            linearMove[i] =
                                sign * penetration * (linearInertia[i] / totalInertia);

                            // To avoid angular projections that are too great (when mass is large
                            // but inertia tensor is small) limit the angular move.
                            Vector3 projection = relativeContactPosition[i];
                            projection += (
                                ContactNormal *
                                -Vector3.Dot(relativeContactPosition[i], ContactNormal)
                                );

                            // Use the small angle approximation for the sine of the angle (i.e.
                            // the magnitude would be sine(angularLimit) * projection.magnitude
                            // but we approximate sine(angularLimit) to angularLimit).
                            float maxMagnitude = angularLimit * projection.Length();

                            if (angularMove[i] < -maxMagnitude)
                            {
                                float totalMove = angularMove[i] + linearMove[i];
                                angularMove[i] = -maxMagnitude;
                                linearMove[i] = totalMove - angularMove[i];
                            }
                            else if (angularMove[i] > maxMagnitude)
                            {
                                float totalMove = angularMove[i] + linearMove[i];
                                angularMove[i] = maxMagnitude;
                                linearMove[i] = totalMove - angularMove[i];
                            }

                            // We have the linear amount of movement required by turning
                            // the rigid body (in angularMove[i]). We now need to
                            // calculate the desired rotation to achieve that.
                            if (angularMove[i] != 0)
                            {
                                // Work out the direction we'd like to rotate in.
                                Vector3 targetAngularDirection =
                                    Vector3.Cross(relativeContactPosition[i], ContactNormal);

                                Matrix inverseInertiaTensor;
                                inverseInertiaTensor = bodies[i].GetInverseInertiaTensorWorld();

                                // Work out the direction we'd need to rotate to achieve that
                                angularChange[i] =
                                    Matrix2.transform(inverseInertiaTensor, targetAngularDirection) * (angularMove[i] / angularInertia[i]);
                            }
                            // Velocity change is easier - it is just the linear movement
                            // along the contact normal.
                            linearChange[i] = ContactNormal * linearMove[i];

                            // Now we can start to apply the values we've calculated.
                            // Apply the linear movement
                            Vector3 pos = bodies[i].GetPosition();
                            //     particle[i].getPosition(&pos);
                            pos += (ContactNormal * linearMove[i]);
                            bodies[i].SetPosition(pos);

                            // And the change in orientation

                            float orient = bodies[i].GetOrientation();
                            orient += angularChange[i].Z;
                            bodies[i].SetOrientation(orient);
                        }
                }
            }
            [Serializable]
            static public class ContactResolver
            {
                static public void ResolveContacts(List<Contact> contacts, int numContacts, float duration)
                {
                    // Make sure we have something to do.
                    if (numContacts == 0) return;

                    // Prepare the contacts for processing
                    PrepareContacts(contacts, numContacts, duration);

                    // Resolve the interpenetration problems with the contacts.
                    AdjustPositions(contacts, numContacts, duration);

                    // Resolve the velocity problems with the contacts.
                    AdjustVelocities(contacts, numContacts, duration);
                }
                static public void PrepareContacts(List<Contact> contacts, float numContacts, float duration)
                {
                    // Generate contact velocity and axis information.

                    for (int i = 0; i < numContacts; i++)
                    {
                        // Calculate the internal contact data (inertia, basis, etc).
                        contacts[i].CalculateInternals(duration);
                    }
                }
                static public void AdjustPositions(List<Contact> c, int numContacts, float duration)
                {
                    int i, index;
                    Vector3[] linearChange = new Vector3[2];
                    Vector3[] angularChange = new Vector3[2];
                    float max=0.9f;
                    Vector3 deltaPosition;

                    // iteratively resolve interpenetrations in order of severity.
                    int positionIterationsUsed = 0, positionIterations = numContacts * 2;
                    while (positionIterationsUsed < positionIterations)
                    {
                        // Find biggest penetration
                        max = 0.1f;//positionEpsilon;
                        index = numContacts;
                        for (i = 0; i < numContacts; i++)
                        {
                            if (c[i].Penetration > max)
                            {
                                max = c[i].Penetration;
                                index = i;
                            }
                        }
                        if (index == numContacts) break;

                        // Match the awake state at the contact
                        c[index].MatchAwakeState();

                        // Resolve the penetration.
                        c[index].ApplyPositionChange(linearChange, angularChange, max);

                        // Again this action may have changed the penetration of other
                        // bodies, so we update contacts.
                        for (i = 0; i < numContacts; i++)
                        {
                            // Check each body in the contact
                            for (int b = 0; b < 2; b++) if (c[i].bodies[b] != null)
                                {
                                    // Check for a match with each body in the newly
                                    // resolved contact
                                    for (int d = 0; d < 2; d++)
                                    {
                                        if (c[i].bodies[b] == c[index].bodies[d])
                                        {
                                            deltaPosition = linearChange[d] +
                                                Vector3.Cross(angularChange[d],
                                                    c[i].relativeContactPosition[b]);
                                            // The sign of the change is positive if we're
                                            // dealing with the second body in a contact
                                            // and negative otherwise (because we're
                                            // subtracting the resolution)..
                                            c[i].Penetration += Vector3.Dot(deltaPosition, c[i].ContactNormal) * (b == 1 ? 1 : -1);
                                        }
                                    }
                                }
                        }
                        positionIterationsUsed++;
                    }
                }
                static public void AdjustVelocities(List<Contact> c, int numContacts, float duration)
                {
                    Vector3[] velocityChange = new Vector3[2];
                    Vector3[] rotationChange = new Vector3[2];
                    Vector3 deltaVel;

                    // iteratively handle impacts in order of severity.
                    int velocityIterationsUsed = 0, velocityIterations = numContacts * 2;
                    while (velocityIterationsUsed < velocityIterations)
                    {
                        // Find contact with maximum magnitude of probable velocity change.
                        float max = 0.9f;//velocityEpsilon;
                        int index = numContacts;
                        for (int i = 0; i < numContacts; i++)
                        {
                            if (c[i].desiredDeltaVelocity > max)
                            {
                                max = c[i].desiredDeltaVelocity;
                                index = i;
                            }
                        }
                        if (index == numContacts) break;

                        // Match the awake state at the contact
                        c[index].MatchAwakeState();

                        // Do the resolution on the contact that came out top.
                        c[index].ApplyVelocityChange(velocityChange, rotationChange);

                        // With the change in velocity of the two bodies, the update of
                        // contact velocities means that some of the relative closing
                        // velocities need recomputing.
                        for (int i = 0; i < numContacts; i++)
                        {
                            // Check each body in the contact
                            for (int b = 0; b < 2; b++) if (c[i].bodies[b] != null)
                                {
                                    // Check for a match with each body in the newly
                                    // resolved contact
                                    for (int d = 0; d < 2; d++)
                                    {
                                        if (c[i].bodies[b] == c[index].bodies[d])
                                        {
                                            deltaVel = velocityChange[d] +
                                                Vector3.Cross(rotationChange[d],
                                                    c[i].relativeContactPosition[b]);
                                            // The sign of the change is negative if we're dealing
                                            // with the second body in a contact.
                                            c[i].contactVelocity +=
                                                Matrix2.transformTranspose(c[i].contactToWorld, deltaVel)
                                                * (b == 1 ? -1 : 1);
                                            c[i].CalculateDesiredDeltaVelocity(duration);
                                        }
                                    }
                                }
                        }
                        velocityIterationsUsed++;
                    }
                }
            }
        }
    }
    
}
