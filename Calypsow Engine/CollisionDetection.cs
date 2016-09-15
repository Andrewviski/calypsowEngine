using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

using Calypsow_Engine.Utility;
using Calypsow_Engine.Rigid;
using Calypsow_Engine.Rigid.CollisionResolution;

namespace Calypsow_Engine
{
    namespace Rigid
    {
        namespace CollisionDetection_Coarse
        {
            //class PotentialContact
            //{
            //    public RigidBody[] body = new RigidBody[2];
            //}
            //public class BoundingCircle
            //{
            //    Vector center;
            //    double radius;
            //    public BoundingCircle(Vector center, double radius)
            //    {
            //        this.center = center;
            //        this.radius = radius;
            //    }
            //    public BoundingCircle(BoundingCircle one, BoundingCircle two)
            //    {
            //        this.center = new Vector((one.center.X + two.center.X) / 2, (one.center.Y + two.center.Y) / 2);
            //        this.radius = ((one.center - two.center).Magnitude() + one.radius + two.radius) / 2;
            //    }
            //    public bool OverLaps(BoundingCircle other)
            //    {
            //        double distanceSquared = (center - other.center).SquareMagnitude();
            //        return distanceSquared < (radius + other.radius) * (radius + other.radius);
            //    }
            //    public double GetSize()
            //    {
            //        return Math.PI * (this.radius * this.radius);
            //    }
            //    public double GetGrowth(BoundingCircle newvolume)
            //    {
            //        BoundingCircle temp = new BoundingCircle(this, newvolume);
            //        return (temp.GetSize() - this.GetSize());
            //    }
            //}
            //public class BVHNode
            //{
            //    BVHNode[] children = new BVHNode[2];
            //    BoundingCircle Volume;
            //    RigidBody body;
            //    public BVHNode(BoundingCircle vol, RigidBody rb)
            //    {
            //        this.body = rb;
            //        this.Volume = vol;
            //        children[0] = null;
            //        children[1] = null;
            //    }
            //    public BVHNode(RigidBody rb)
            //    {
            //        this.body = rb;
            //        if (rb is Circle)
            //        {
            //            Circle temp = rb as Circle;
            //            this.Volume = new BoundingCircle(temp.CenterOfMass.Position, temp.Radius);
            //        }
            //        children[0] = null;
            //        children[1] = null;
            //    }
            //    bool IsLeaf()
            //    {
            //        return body != null;
            //    }
            //    void ReCalculateBoundingVolume()
            //    {
            //        Volume = new BoundingCircle(children[0].Volume, children[1].Volume);
            //    }
            //    bool OverLaps(BVHNode other)
            //    {
            //        return this.Volume.OverLaps(other.Volume);
            //    }
            //    int GetPotintialContacts(ref PotentialContact[] contacts, int limit)
            //    {
            //        if (this.IsLeaf() || limit == 0)
            //            return 0;
            //        return children[0].GetPotintialContactsWith(children[1], ref contacts, limit);
            //    }
            //    int GetPotintialContactsWith(BVHNode other, ref PotentialContact[] contacts, int limit, int i = 0)
            //    {
            //        //if we don't have overlap or if we have no room 
            //        // to report contacts
            //        if (!this.OverLaps(other) || limit == 0)
            //        {
            //            contacts = null;
            //            return 0;
            //        }
            //        //if both were leaf  nodes, then we have a potintial contact
            //        if (this.IsLeaf() && other.IsLeaf())
            //        {
            //            contacts[i].body[0] = this.body;
            //            contacts[i].body[0] = other.body;
            //            return 1;
            //        }
            //        //determine which node to dive into, if it's is a leaf, we dive into the other.
            //        // if both were branches, the we use the one with the largest size
            //        if (other.IsLeaf() || (!this.IsLeaf() && this.Volume.GetSize() > other.Volume.GetSize()))
            //        {
            //            //dive into yourself
            //            int count = this.children[0].GetPotintialContactsWith(other, ref contacts, limit, i);
            //            //check if we can do the other side too
            //            if (limit > count)
            //            {
            //                return count + this.children[1].GetPotintialContactsWith(other, ref contacts, (limit - count), (i + count));
            //            }
            //            else
            //            {
            //                return count;
            //            }
            //        }
            //        else
            //        {
            //            //dive into the other node
            //            int count = this.GetPotintialContactsWith(other.children[0], ref contacts, limit, i);
            //            //check if we can do the other side too
            //            if (limit > count)
            //            {
            //                return count + this.GetPotintialContactsWith(other.children[1], ref contacts, (limit - count), (i + count));
            //            }
            //            else
            //            {
            //                return count;
            //            }
            //        }
            //    }
            //    void Insert(RigidBody newbody, BoundingCircle newvolume)
            //    {
            //        //if we are a leaf , then the only option is to spawn two children
            //        //and place the new body in one
            //        if (this.IsLeaf())
            //        {
            //            //child one is a copy of us
            //            children[0] = new BVHNode(this.Volume, this.body);
            //            //child two holds the new body
            //            children[1] = new BVHNode(newvolume, newbody);
            //            //we loosen the body (we're no longer a leaf)
            //            this.body = null;
            //            ReCalculateBoundingVolume();
            //        }
            //        //otherwise we need to work out which child gets to keep
            //        //the inserted body. we give it to whoever grow the least to incorporate it.
            //        else
            //        {
            //            if (children[0].Volume.GetGrowth(newvolume) < children[1].Volume.GetGrowth(newvolume))
            //            {
            //                children[0].Insert(newbody, newvolume);
            //            }
            //            else
            //            {
            //                children[1].Insert(newbody, newvolume);
            //            }
            //        }
            //    }
            //    void Remove() { /*TODO*/ }
            //}
        }
        namespace CollisionDetection_Fine
        {
            [Serializable]
            public class CollisionData
            {
                public List<Contact> Contacts;
                public int ContactsLeft;
                public int ContactUsed;
                public int index;

                public CollisionData(int contactsLeft)
                {
                    this.Contacts = new List<Contact>();
                    this.ContactsLeft = contactsLeft;
                    this.ContactUsed = 0;
                }
                public bool HasMoreContacts()
                {
                    return ContactsLeft < 0;
                }
                public void Reset(int max)
                {
                    Contacts.Clear();
                    ContactsLeft = max;
                }
                public void AddContacts(int count)
                {
                    // Reduce the number of contacts remaining, add number used
                    ContactsLeft -= count;
                    ContactUsed += count;
                    // Move the array forward
                    index++;
                }
            }
            [Serializable]
            public abstract class CollisionPrimitive
            {
                /**
                 * The rigid body that is represented by this primitive.
                 */
                public RigidBody body;
            }
            [Serializable]
            public class CollisionPlane : CollisionPrimitive
            {
                /**
                 * the plane point
                 */
                public Vector3 point;
                /**
                 * the tangant (orientation) of the plane
                 */
                public float tangent;

                //line attributes
                public float a;
                public float b;
                public float c;
                public Vector3 direction;
                public Vector3 originUsed;

                public CollisionPlane(Vector3 point, float tangent,Vector3 tangentVector)
                {
                    this.direction = new Vector3();
                    this.direction.X = tangentVector.Y;
                    this.direction.Y = -tangentVector.X;
                    this.direction.Z = 0;
                    this.direction.Normalize();

                    if (float.IsNaN(tangent))
                    {
                        this.point = point;
                        this.tangent = float.NaN;
                        this.body = null;

                        this.a = 0;
                        this.b = 1;
                        this.c = -point.Y;
                    }
                    else
                    {
                        this.body = null;
                        this.point = point;
                        this.tangent = tangent;

                        float alpha = this.point.Y - tangent * this.point.X;

                        this.c = -alpha;
                        this.b = 1;
                        this.a = -tangent;
                    }
                    originUsed = direction + this.point;    
                }
                public float signedDistance(Vector3 somepoint)
                {
                    float temp = putInLine(somepoint);
                    temp /= (float)Math.Sqrt(this.a * this.a + this.b * this.b);
                    return temp;
                }
                public float distance(Vector3 somepoint)
                {
                    return Math.Abs(signedDistance(somepoint));
                }
                public float putInLine(Vector3 somepoint)
                {
                    float temp = 0;
                    temp = this.a * somepoint.X + this.b * somepoint.Y + this.c;
                    return temp;
                }
            }
            [Serializable]
            public class CollisionCircle : CollisionPrimitive
            {
                /**
                 * The radius of the Circle.
                 */
                public float radius;
                public CollisionCircle(float r)
                {
                    this.radius = r;
                }
            }
            [Serializable]
            public class CollisionBox : CollisionPrimitive
            {
                /**
                 * Holds the half-sizes of the box along each of its local axes.
                 */
                public Vector3 halfSize;
                public CollisionBox(Vector3 halfSize)
                {
                    this.halfSize = halfSize;
                }
            }
            [Serializable]
            public static class CollisionDetector
            {
                public static bool BoxAndHalfSpace(CollisionBox box, CollisionPlane plane, ref CollisionData data)
                {
                    bool isCollision = false;
                    // Make sure we have contacts
                    if (data.ContactsLeft <= 0) return false;

                    // Cache the box position
                    Vector3 positionOne = box.body.GetPosition();

                    Box temp = box.body as Box;

                    Vector3[] points = new Vector3[]{ 
                     positionOne + Vector3.Multiply(temp.xAxis,temp.HalfSizeWidth) + Vector3.Multiply(temp.yAxis,temp.HalfSizeheight) ,
                     positionOne + Vector3.Multiply(temp.xAxis,-temp.HalfSizeWidth) + Vector3.Multiply(temp.yAxis,temp.HalfSizeheight) ,
                     positionOne + Vector3.Multiply(temp.xAxis,temp.HalfSizeWidth) + Vector3.Multiply(temp.yAxis,-temp.HalfSizeheight) ,
                     positionOne + Vector3.Multiply(temp.xAxis,-temp.HalfSizeWidth) + Vector3.Multiply(temp.yAxis,-temp.HalfSizeheight) 
                    };


                    float originDueToHalfSpace = plane.putInLine(plane.originUsed);

                    for (int i = 0; i < 4; i++)
                    {
                        float pointDueToHalfSpace = plane.putInLine(points[i]);

                        if ((points[i].Y>=plane.point.Y)&&((originDueToHalfSpace < 0 && pointDueToHalfSpace > 0) || (originDueToHalfSpace > 0 && pointDueToHalfSpace < 0)))
                        {
                            isCollision = true;
                            // Create the contact - it has a normal in the plane direction.

                            Contact c = new Contact();
                            c.ContactNormal = plane.direction;
                            c.Penetration = plane.distance(points[i]);
                            c.ContactPoint = points[i];
                            c.setBodyData(box.body, null, Tables.GetFriction(box.body), Tables.GetRestitution(box.body));
                            c.contactToWorld.M11 = plane.direction.X;
                            c.contactToWorld.M12 = -plane.direction.Y;
                            c.contactToWorld.M21 = plane.direction.Y;
                            c.contactToWorld.M22 = plane.direction.X;

                            data.Contacts.Add(c);
                            box.body.SetCanSleep(true);
                            box.body.hitGround = true;
                            data.AddContacts(1);
                        }
                    }
                    return isCollision;
                }
                public static bool CircleAndHalfSpace(CollisionCircle sphere, CollisionPlane plane, ref CollisionData data)
                {
                    // Make sure we have contacts
                    if (data.ContactsLeft <= 0) return false;
         
                    // Cache the box position
                    Vector3 positionOne = sphere.body.GetPosition();

                    Circle temp = sphere.body as Circle;

                    Vector3 offset = Vector3.Multiply(plane.direction, -temp.Radius);

                    float originDueToHalfSpace = plane.putInLine(plane.originUsed);

                    Vector3 point = positionOne + offset;
                    float pointDueToHalfSpace = plane.putInLine(point);

                    if ((point.Y >=plane.point.Y)&&((originDueToHalfSpace < 0 && pointDueToHalfSpace > 0) || (originDueToHalfSpace > 0 && pointDueToHalfSpace < 0)))
                    {
                        // Create the contact - it has a normal in the plane direction.
                        Contact c = new Contact();
                        c.ContactNormal = plane.direction;
                        c.Penetration = plane.distance(point);
                        c.ContactPoint = point;
                        c.setBodyData(sphere.body, null, Tables.GetFriction(sphere.body)-0.35f, Tables.GetRestitution(sphere.body));
                        c.contactToWorld.M11 = plane.direction.X;
                        c.contactToWorld.M12 = -plane.direction.Y;
                        c.contactToWorld.M21 = plane.direction.Y;
                        c.contactToWorld.M22 = plane.direction.X;

                        data.Contacts.Add(c);
                        sphere.body.SetCanSleep(true);
                        sphere.body.hitGround = true;
                        data.AddContacts(1);
                        return true;
                    }
                    return false;
                }
                public static bool CircleAndCircle(CollisionCircle one, CollisionCircle two, ref CollisionData data)
                {
                    // Make sure we have contacts
                    if (data.ContactsLeft <= 0) return false;

                    // Cache the sphere positions
                    Vector3 positionOne = one.body.GetPosition();
                    Vector3 positionTwo = two.body.GetPosition();

                    // Find the vector between the objects
                    Vector3 midline = positionOne - positionTwo;
                    float size = midline.Length();

                    // See if it is large enough.
                    if (size <= 0.0f || size >= one.radius + two.radius)
                    {
                        return false;
                    }

                    // We manually create the normal, because we have the
                    // size to hand.
                    Vector3 normal = midline * (((float)1.0) / size);



                    Contact c = new Contact();
                    c.ContactNormal = normal;
                    c.ContactPoint = positionTwo + midline * (float)0.5;
                    c.Penetration = (one.radius + two.radius - size);
                    c.setBodyData(one.body, two.body, Tables.GetFriction(one.body,two.body), Tables.GetRestitution(one.body,two.body));
                    c.contactToWorld.M11 = normal.X;
                    c.contactToWorld.M12 = -normal.Y;
                    c.contactToWorld.M21 = normal.Y;
                    c.contactToWorld.M22 = normal.X;
                    one.body.SetCanSleep(true);
                    two.body.SetCanSleep(true);
                    data.Contacts.Add(c);
                    data.AddContacts(1);
                    
                    return true;
                }
                public static bool BoxAndBox(CollisionBox one, CollisionBox two, ref CollisionData data)
                {
                    //if (!IntersectionTestsboxAndBox(one, two)) return false;

                    // Find the vector between the two centres
                    Vector3 toCentre = two.body.GetPosition() - one.body.GetPosition();

                    // We start assuming there is no contact
                    float pen = float.MaxValue;
                    int best = 0xffffff;

                    // Now we check each axes, returning if it gives us
                    // a separating axis, and keeping track of the axis with
                    // the smallest penetration otherwise.

                    if (!TryAxis(one, two, ((Box)one.body).xAxis, toCentre, 0, ref pen, ref best)) return false;
                    if (!TryAxis(one, two, ((Box)one.body).yAxis, toCentre, 1, ref pen, ref best)) return false;

                    if (!TryAxis(one, two, ((Box)two.body).xAxis, toCentre, 2, ref pen, ref best)) return false;
                    if (!TryAxis(one, two, ((Box)two.body).yAxis, toCentre, 3, ref pen, ref best)) return false;


                    // Store the best axis-major, in case we run into almost
                    // parallel edge collisions later
                    int bestSingleAxis = best;

                    // Make sure we've got a result.

                    if (best != 0xffffff)
                    {

                        // We now know there's a collision, and we know which
                        // of the axes gave the smallest penetration. We now
                        // can deal with it in different ways depending on
                        // the case.
                        if (best < 2)
                        {
                            // We've got a vertex of box two on a face of box one.
                            FillPointFaceBoxBox(one, two, toCentre, ref data, best, pen);
                            data.AddContacts(1);
                        }
                        else if (best < 4)
                        {
                            // We've got a vertex of box one on a face of box two.
                            // We use the same algorithm as above, but swap around
                            // one and two (and therefore also the vector between their
                            // centres).
                            FillPointFaceBoxBox(two, one, toCentre * -1.0f, ref data, best - 2, pen);
                            
                            data.AddContacts(1);
                        }
                        one.body.SetCanSleep(true);
                        two.body.SetCanSleep(true);
                        return true;
                    }
                    return false;
                }
                public static bool BoxAndCircle(CollisionBox box, CollisionCircle sphere, ref CollisionData data)
                {
                    // Transform the centre of the sphere into box coordinates
                    Vector3 centre = sphere.body.GetPosition();
                    Vector3 relCentre = centre - box.body.GetPosition();

                    relCentre = Matrix2.M_V(relCentre, -box.body.GetOrientation());

                    // Early out check to see if we can exclude the contact
                    if (Math.Abs(relCentre.X) - sphere.radius > box.halfSize.X || Math.Abs(relCentre.Y) - sphere.radius > box.halfSize.Y)
                        return false;

                    Vector3 closestPt = new Vector3(0, 0, 0);
                    float dist;

                    // Clamp each coordinate to the box.
                    dist = relCentre.X;
                    if (dist > box.halfSize.X) dist = box.halfSize.X;
                    if (dist < -box.halfSize.X) dist = -box.halfSize.X;
                    closestPt.X = dist;

                    dist = relCentre.Y;
                    if (dist > box.halfSize.Y) dist = box.halfSize.Y;
                    if (dist < -box.halfSize.Y) dist = -box.halfSize.Y;
                    closestPt.Y = dist;

                    // Check we're in contact
                    Vector3 temp1 = closestPt - relCentre;
                    if (temp1 == Vector3.Zero)
                        return true;
                    float temp2 = temp1.Length();
                    double temp3 = temp2;
                    dist = (float)Math.Pow(temp3, 2);
                    if (dist > sphere.radius * sphere.radius) return false;

                    // Compile the contact
                    Vector3 closestPtWorld = closestPt;

                    closestPtWorld = Matrix2.M_V(closestPtWorld, box.body.GetOrientation());
                    closestPtWorld += box.body.GetPosition();

                    // Create the contact data

                    Vector3 temp = closestPtWorld - centre;
                    temp.Normalize();

                    Contact c = new Contact();

                    c.ContactNormal = temp;
                    c.Penetration = (float)(sphere.radius - Math.Sqrt(dist));
                    c.ContactPoint = closestPtWorld;
                    if (((Box)box.body).JointedCircles.Contains(sphere.body as Circle))
                    {
                        c.setBodyData(box.body, sphere.body, 0, 0);
                    }
                    else
                    {
                        c.setBodyData(box.body, sphere.body, Tables.GetFriction(box.body, sphere.body), Tables.GetRestitution(box.body, sphere.body));
                    }
                    
                    c.contactToWorld.M11 = c.ContactNormal.X;
                    c.contactToWorld.M12 = -c.ContactNormal.Y;
                    c.contactToWorld.M21 = c.ContactNormal.Y;
                    c.contactToWorld.M22 = c.ContactNormal.X;

                    sphere.body.SetCanSleep(true);
                    box.body.SetCanSleep(true);
                    data.AddContacts(1);
                    data.Contacts.Add(c);

                    return true;
                }
                static float TransformToAxis(CollisionBox box, Vector3 axis)
                {
                    return box.halfSize.X * (float)Math.Abs(Vector3.Dot(axis, ((Box)box.body).xAxis)) + box.halfSize.Y * (float)Math.Abs(Vector3.Dot(axis, ((Box)box.body).yAxis));
                }
                //static bool IntersectionTestboxAndHalfSpace(CollisionBox box, CollisionPlane plane)
                //{
                //    // Work out the projected radius of the box onto the plane direction
                //    float projectedRadius = TransformToAxis(box, plane.direction);

                //    // Work out how far the box is from the origin
                //    float boxDistance = Vector3.Dot(plane.direction, box.body.GetPosition()) - projectedRadius;

                //    // Check for the intersection
                //    return boxDistance <= plane.offset;
                //}
                public static bool OverlapOnAxis(CollisionBox one, CollisionBox two, Vector3 axis, Vector3 toCentre)
                {
                    // Project the half-size of one onto axis
                    float oneProject = TransformToAxis(one, axis);
                    float twoProject = TransformToAxis(two, axis);

                    // Project this onto the axis
                    float distance = Math.Abs(Vector3.Dot(toCentre, axis));

                    // Check for overlap
                    return (distance < oneProject + twoProject);
                }
                public static bool IntersectionTestsboxAndBox(CollisionBox one, CollisionBox two)
                {
                    // Find the vector between the two centres
                    Vector3 toCentre = two.body.GetPosition() - one.body.GetPosition();
                    return (
                        // Check on box one's axes first
                        OverlapOnAxis(one, two, ((Box)one.body).xAxis, toCentre) &&
                        OverlapOnAxis(one, two, ((Box)one.body).yAxis, toCentre) &&
                        // And on two's
                        OverlapOnAxis(one, two, ((Box)two.body).xAxis, toCentre) &&
                        OverlapOnAxis(one, two, ((Box)two.body).yAxis, toCentre)
                    );
                }
                /*
                 * This function checks if the two boxes overlap
                 * along the given axis, returning the ammount of overlap.
                 * The final parameter toCentre
                 * is used to pass in the vector between the boxes centre
                 * points, to avoid having to recalculate it each time.
                 */
                static float PenetrationOnAxis(CollisionBox one, CollisionBox two, Vector3 axis, Vector3 toCentre)
                {
                    // Project the half-size of one onto axis
                    float oneProject = TransformToAxis(one, axis);
                    float twoProject = TransformToAxis(two, axis);

                    // Project this onto the axis
                    float distance = Math.Abs(Vector3.Dot(toCentre, axis));

                    // Return the overlap (i.e. positive indicates
                    // overlap, negative indicates separation).
                    return oneProject + twoProject - distance;
                }
                static bool TryAxis(CollisionBox one, CollisionBox two, Vector3 axis, Vector3 toCentre, int index, ref float smallestPenetration, ref int smallestCase)
                {
                    // Make sure we have a normalized axis, and don't check almost parallel axes
                    if (axis.Length() * axis.Length() < 0.00001) return true;
                    axis.Normalize();

                    float penetration = PenetrationOnAxis(one, two, axis, toCentre);

                    if (penetration < 0) return false;
                    if (penetration < smallestPenetration)
                    {
                        smallestPenetration = penetration;
                        smallestCase = index;
                    }
                    return true;
                }
                static void FillPointFaceBoxBox(CollisionBox one, CollisionBox two, Vector3 toCentre, ref CollisionData data, int best, float pen)
                {
                    // This method is called when we know that a vertex from
                    // box two is in contact with box one.


                    // We know which axis the collision is on (i.e. best),
                    // but we need to work out which of the two faces on
                    // this axis.
                    Vector3 normal;
                    if (best == 0)
                    {
                        normal = ((Box)one.body).xAxis;
                    }
                    else
                    {
                        normal = ((Box)one.body).yAxis;
                    }

                    if (Vector3.Dot(normal, toCentre) > 0)
                    {
                        normal = normal * -1.0f;
                    }

                    // Work out which vertex of box two we're colliding with.
                    // Using toCentre doesn't work!
                    Vector3 vertex = two.halfSize;

                    if (Vector3.Dot(((Box)two.body).xAxis, normal) < 0)
                        vertex.X = -vertex.X;
                    if (Vector3.Dot(((Box)two.body).yAxis, normal) < 0)
                        vertex.Y = -vertex.Y;

                    vertex = Matrix2.M_V(vertex, two.body.GetOrientation());
                    vertex += two.body.GetPosition();

                    Contact c = new Contact();
                    c.contactToWorld.M11 = normal.X;
                    c.contactToWorld.M12 = -normal.Y;
                    c.contactToWorld.M21 = normal.Y;
                    c.contactToWorld.M22 = normal.X;

                    // Create the contact data
                    c.ContactNormal = normal;
                    c.Penetration = pen;
                    c.ContactPoint = vertex;
                    c.setBodyData(one.body, two.body, Tables.GetFriction(one.body,two.body),Tables.GetRestitution(one.body,two.body));
                    data.Contacts.Add(c);
                }
            }
        }
    }
}
