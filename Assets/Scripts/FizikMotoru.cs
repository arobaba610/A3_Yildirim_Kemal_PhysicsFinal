using System;
using System.Collections.Generic;
using UnityEngine;

public class FizikMotoru : MonoBehaviour
{
    static FizikMotoru instance = null;
    public static FizikMotoru Instance
    {
        get
        {
            if (instance == null)
            {
                instance = FindObjectOfType<FizikMotoru>();
            }
            return instance;
        }
    }



    public List<FizikObject> objects = new List<FizikObject>();
    public float dt = 0.02f;
    public Vector3 AccelerationGravity = new Vector3(0, -10, 0);


    private static float[,] restitutionTable = new float[,]
{
    // Steel, Wood, Rubber, Cloth, Stone
    { 0.9f, 0.7f, 0.6f, 0.4f, 0.8f }, // Steel
    { 0.7f, 0.5f, 0.4f, 0.3f, 0.6f }, // Wood
    { 0.6f, 0.4f, 0.8f, 0.2f, 0.5f }, // Rubber
    { 0.4f, 0.3f, 0.2f, 0.1f, 0.3f }, // Cloth
    { 0.8f, 0.6f, 0.5f, 0.3f, 0.7f }  // Stone
};

    private float GetRestitution(SurfaceMaterial matA, SurfaceMaterial matB)
    {
        int indexA = (int)matA;
        int indexB = (int)matB;
        return restitutionTable[indexA, indexB];
    }


    void FixedUpdate()
    {
        dt = Time.fixedDeltaTime; // Ensure accurate timestep calculation
        CleanUpObjects();
        UpdatePhysics();
        ResetVisuals();
        DetectCollisions();
    }

    private void CleanUpObjects()
    {
        objects.RemoveAll(obj => obj == null);
    }

    private void UpdatePhysics()
    {
        foreach (FizikObject obj in objects)
        {
            if (obj.isStatic) continue;

            // Apply Gravity
            obj.velocity += AccelerationGravity * obj.gravityScale * dt;

            // Apply Drag
            Vector3 dragForce = -obj.drag * obj.velocity.sqrMagnitude * obj.velocity.normalized;
            obj.velocity += dragForce * dt;

            // Update Position
            obj.transform.position += obj.velocity * dt;
        }
    }

    private void UpdatePosition(FizikObject obj)
    {
        obj.transform.position += obj.velocity * dt;
    }


    private void ResetVisuals()
    {
        foreach (FizikObject obj in objects)
        {
            if (obj != null && obj.GetComponent<Renderer>() != null)
            {
                obj.GetComponent<Renderer>().material.color = Color.white;
            }
        }
    }

    private void DetectCollisions()
    {
        for (int iA = 0; iA < objects.Count; iA++)
        {
            FizikObject objectA = objects[iA];
            if (objectA == null) continue;

            for (int iB = iA + 1; iB < objects.Count; iB++)
            {
                FizikObject objectB = objects[iB];
                if (objectB == null) continue;

                if (HandleCollision(objectA, objectB))
                {
                    objectA.GetComponent<Renderer>().material.color = Color.red;
                    objectB.GetComponent<Renderer>().material.color = Color.red;
                    Debug.DrawLine(objectA.transform.position, objectB.transform.position, Color.red, 0.1f);
                }
            }
        }
    }

    private bool HandleCollision(FizikObject objectA, FizikObject objectB)
    {
        bool isOverlapping = false;

        switch (objectA.shape.GetShape())
        {
            case FizziksShape.Shape.Sphere when objectB.shape.GetShape() == FizziksShape.Shape.Sphere:
                isOverlapping = CollideSpheres(objectA, objectB);
                break;

            case FizziksShape.Shape.Sphere when objectB.shape.GetShape() == FizziksShape.Shape.Plane:
                isOverlapping = CollideSpherePlane((FizziksShapeSphere)objectA.shape, (FizziksShapePlane)objectB.shape, objectA);
                break;

            case FizziksShape.Shape.Plane when objectB.shape.GetShape() == FizziksShape.Shape.Sphere:
                isOverlapping = CollideSpherePlane((FizziksShapeSphere)objectB.shape, (FizziksShapePlane)objectA.shape, objectB);
                break;

            case FizziksShape.Shape.Sphere when objectB.shape.GetShape() == FizziksShape.Shape.Halfspace:
                isOverlapping = CollideSphereHalfspace((FizziksShapeSphere)objectA.shape, (FizziksShapeHalfspace)objectB.shape, objectA);
                break;

            case FizziksShape.Shape.Halfspace when objectB.shape.GetShape() == FizziksShape.Shape.Sphere:
                isOverlapping = CollideSphereHalfspace((FizziksShapeSphere)objectB.shape, (FizziksShapeHalfspace)objectA.shape, objectB);
                break;

            case FizziksShape.Shape.AABB when objectB.shape.GetShape() == FizziksShape.Shape.AABB:
                isOverlapping = CollideAABBs((FizziksShapeAABB)objectA.shape, (FizziksShapeAABB)objectB.shape, objectA, objectB);
                break;

            case FizziksShape.Shape.AABB when objectB.shape.GetShape() == FizziksShape.Shape.Sphere:
                isOverlapping = CollideSphereAABB((FizziksShapeSphere)objectB.shape, (FizziksShapeAABB)objectA.shape, objectB);
                break;

            case FizziksShape.Shape.Sphere when objectB.shape.GetShape() == FizziksShape.Shape.AABB:
                isOverlapping = CollideSphereAABB((FizziksShapeSphere)objectA.shape, (FizziksShapeAABB)objectB.shape, objectA);
                break;

            case FizziksShape.Shape.AABB when objectB.shape.GetShape() == FizziksShape.Shape.Plane:
                isOverlapping = CollideAABBPlane((FizziksShapeAABB)objectA.shape, (FizziksShapePlane)objectB.shape, objectA);
                break;

            case FizziksShape.Shape.Plane when objectB.shape.GetShape() == FizziksShape.Shape.AABB:
                isOverlapping = CollideAABBPlane((FizziksShapeAABB)objectB.shape, (FizziksShapePlane)objectA.shape, objectB);
                break;
        }

        return isOverlapping;
    }



    public static bool SweptAABB(FizziksShapeAABB aabbA, FizziksShapeAABB aabbB, Vector3 velocityA, Vector3 velocityB, out float collisionTime)
    {
        collisionTime = float.MaxValue;

        // Relative velocity
        Vector3 relativeVelocity = velocityA - velocityB;

        // Define entry and exit times along each axis
        float tMin = 0.0f, tMax = 1.0f; // Time range within the current frame

        for (int i = 0; i < 3; i++) // X, Y, Z axes
        {
            float invVelocity = relativeVelocity[i] != 0.0f ? 1.0f / relativeVelocity[i] : float.PositiveInfinity;

            float entry = (aabbB.min[i] - aabbA.max[i]) * invVelocity;
            float exit = (aabbB.max[i] - aabbA.min[i]) * invVelocity;

            if (entry > exit) (entry, exit) = (exit, entry); // Swap if inverted

            tMin = Mathf.Max(tMin, entry);
            tMax = Mathf.Min(tMax, exit);

            if (tMin > tMax) return false; // No collision
        }

        collisionTime = tMin;
        return tMin <= 1.0f; // Collision occurs within the frame
    }


    private void ApplyDrag(FizikObject objectA)
    {
        float dragCoefficient = objectA.drag;
        Vector3 dragForce = -dragCoefficient * objectA.velocity.sqrMagnitude * objectA.velocity.normalized;
        objectA.velocity += dragForce * dt;
    }

    public static bool CollideSpheres(FizikObject objectA, FizikObject objectB)
    {
        Vector3 displacement = objectA.transform.position - objectB.transform.position;
        float distance = displacement.magnitude;
        float radiusA = ((FizziksShapeSphere)objectA.shape).radius;
        float radiusB = ((FizziksShapeSphere)objectB.shape).radius;

        float overlap = radiusA + radiusB - distance;

        if (overlap > 0.0f)
        {
            Vector3 collisionNormal = displacement.normalized;

            // Momentum ve hız değişimleri
            Vector3 relativeVelocity = objectA.velocity - objectB.velocity;


            float restitution = (objectA.restitution + objectB.restitution) / 2;

            float impulseMagnitude = (1 + restitution) * Vector3.Dot(relativeVelocity, collisionNormal) /
                                     (1 / objectA.mass + 1 / objectB.mass);

            Vector3 impulse = collisionNormal * impulseMagnitude;

            if (!objectA.isStatic)
                objectA.velocity -= impulse / objectA.mass;

            if (!objectB.isStatic)
                objectB.velocity += impulse / objectB.mass;

            // Çarpışma sonrası pozisyon düzeltmesi (penetrasyonu önleme)
            Vector3 mtv = collisionNormal * overlap * 0.5f;
            if (!objectA.isStatic) objectA.transform.position += mtv;
            if (!objectB.isStatic) objectB.transform.position -= mtv;

            return true;
        }

        return false;
    }



    public bool CollideSpherePlane(FizziksShapeSphere sphere, FizziksShapePlane plane, FizikObject objectA)
    {
        Vector3 planeNormal = plane.Normal().normalized;
        Vector3 planeToSphere = sphere.transform.position - plane.transform.position;
        float distanceToPlane = Vector3.Dot(planeToSphere, planeNormal);

        if (Mathf.Abs(distanceToPlane) < sphere.radius)
        {
            Vector3 mtv = planeNormal * (sphere.radius - Mathf.Abs(distanceToPlane));
            objectA.transform.position += mtv;

            Vector3 normalForce = -planeNormal * AccelerationGravity.magnitude * objectA.mass;
            Vector3 gravityPerpendicular = AccelerationGravity * objectA.gravityScale - Vector3.Project(AccelerationGravity, planeNormal);
            Vector3 frictionForce = -objectA.velocity.normalized * 0.5f * normalForce.magnitude;

            objectA.velocity += (gravityPerpendicular + frictionForce) * dt;
            objectA.velocity -= (1 + 0.8f) * Vector3.Dot(objectA.velocity, planeNormal) * planeNormal;

            return true;
        }

        return false;
    }



    public static bool CollideAABBs(FizziksShapeAABB aabbA, FizziksShapeAABB aabbB, FizikObject objectA, FizikObject objectB)
    {
        // Recalculate bounds to ensure they are up to date
        aabbA.RecalculateBounds();
        aabbB.RecalculateBounds();

        // Check for overlap along each axis
        float overlapX = Mathf.Min(aabbA.max.x - aabbB.min.x, aabbB.max.x - aabbA.min.x);
        float overlapY = Mathf.Min(aabbA.max.y - aabbB.min.y, aabbB.max.y - aabbA.min.y);
        float overlapZ = Mathf.Min(aabbA.max.z - aabbB.min.z, aabbB.max.z - aabbA.min.z);

        if (overlapX > 0 && overlapY > 0 && overlapZ > 0)
        {
            // Determine the axis of least penetration (minimum overlap)
            float[] overlaps = { overlapX, overlapY, overlapZ };
            int minAxis = Array.IndexOf(overlaps, Mathf.Min(overlaps));

            Vector3 collisionNormal = Vector3.zero;

            // Set the MTV direction based on the axis with the least penetration
            switch (minAxis)
            {
                case 0: collisionNormal = Vector3.right * Mathf.Sign(aabbA.min.x - aabbB.min.x); break; // X-axis
                case 1: collisionNormal = Vector3.up * Mathf.Sign(aabbA.min.y - aabbB.min.y); break;   // Y-axis
                case 2: collisionNormal = Vector3.forward * Mathf.Sign(aabbA.min.z - aabbB.min.z); break; // Z-axis
            }

            // Minimum Translation Vector
            Vector3 mtv = collisionNormal * overlaps[minAxis];

            // Apply the MTV to separate the objects
            if (!objectA.isStatic)
                objectA.transform.position += mtv * 0.5f;
            if (!objectB.isStatic)
                objectB.transform.position -= mtv * 0.5f;

            // Resolve velocities (impulse-based response)
            Vector3 relativeVelocity = objectA.velocity - objectB.velocity;
            float impulseMagnitude = Vector3.Dot(relativeVelocity, collisionNormal) * (1 + objectA.restitution) / (1 / objectA.mass + 1 / objectB.mass);
            Vector3 impulse = collisionNormal * impulseMagnitude;

            if (!objectA.isStatic) objectA.velocity -= impulse / objectA.mass;
            if (!objectB.isStatic) objectB.velocity += impulse / objectB.mass;

            return true;
        }

        return false;
    }


    public static bool CollideSphereAABB(FizziksShapeSphere sphere, FizziksShapeAABB aabb, FizikObject sphereObject)
    {
        aabb.RecalculateBounds();

        // Predict the sphere's movement using its velocity
        Vector3 startPosition = sphere.transform.position;
        Vector3 endPosition = startPosition + sphereObject.velocity * FizikMotoru.Instance.dt;

        Vector3 direction = (endPosition - startPosition).normalized;
        float maxDistance = (endPosition - startPosition).magnitude;

        // Step 1: Perform initial collision check at the current position
        Vector3 closestPoint = Vector3.Max(aabb.min, Vector3.Min(startPosition, aabb.max));
        float closestDistance = (closestPoint - startPosition).sqrMagnitude;

        if (closestDistance <= sphere.radius * sphere.radius)
        {
            // Step 2: Calculate collision normal and penetration depth
            Vector3 sphereToClosest = closestPoint - startPosition;
            float distance = sphereToClosest.magnitude;
            Vector3 collisionNormal = distance > 0 ? sphereToClosest.normalized : Vector3.up;
            float penetrationDepth = sphere.radius - distance;

            // Step 3: Resolve collision (push sphere out)
            sphereObject.transform.position -= collisionNormal * penetrationDepth;

            // Reflect velocity to simulate bouncing
            float restitution = sphereObject.restitution;
            sphereObject.velocity = Vector3.Reflect(sphereObject.velocity, collisionNormal) * restitution;

            return true;
        }

        // Step 4: Swept collision detection (raycast-like check)
        Ray ray = new Ray(startPosition, direction);
        float t = 0.0f;

        if (SweptSphereAABB(ray, aabb, sphere.radius, out t) && t <= maxDistance)
        {
            Vector3 collisionPoint = startPosition + direction * t;
            Vector3 collisionNormal = (collisionPoint - closestPoint).normalized;

            // Move the sphere to the collision point
            sphereObject.transform.position = collisionPoint - collisionNormal * sphere.radius;

            // Reflect velocity
            float restitution = sphereObject.restitution;
            sphereObject.velocity = Vector3.Reflect(sphereObject.velocity, collisionNormal) * restitution;

            return true;
        }

        return false;
    }

    // Swept Sphere-AABB Helper
    private static bool SweptSphereAABB(Ray ray, FizziksShapeAABB aabb, float sphereRadius, out float t)
    {
        t = 0.0f;
        float tNear = float.NegativeInfinity, tFar = float.PositiveInfinity;

        for (int i = 0; i < 3; i++)
        {
            float invRayDir = 1.0f / ray.direction[i];
            float t1 = (aabb.min[i] - ray.origin[i] - sphereRadius) * invRayDir;
            float t2 = (aabb.max[i] - ray.origin[i] + sphereRadius) * invRayDir;

            if (t1 > t2) (t1, t2) = (t2, t1);

            tNear = Mathf.Max(tNear, t1);
            tFar = Mathf.Min(tFar, t2);

            if (tNear > tFar || tFar < 0.0f)
                return false; // No collision
        }

        t = tNear;
        return true;
    }




    public static bool CollideAABBPlane(FizziksShapeAABB aabb, FizziksShapePlane plane, FizikObject objectA)
    {
        // Recalculate bounds to ensure they're up-to-date
        aabb.RecalculateBounds();

        // Get the plane normal and distance
        Vector3 planeNormal = plane.Normal().normalized;
        Vector3 planePosition = plane.transform.position;


        // Check each vertex of the AABB against the plane
        Vector3[] vertices = {
        new Vector3(aabb.min.x, aabb.min.y, aabb.min.z),
        new Vector3(aabb.min.x, aabb.min.y, aabb.max.z),
        new Vector3(aabb.min.x, aabb.max.y, aabb.min.z),
        new Vector3(aabb.min.x, aabb.max.y, aabb.max.z),
        new Vector3(aabb.max.x, aabb.min.y, aabb.min.z),
        new Vector3(aabb.max.x, aabb.min.y, aabb.max.z),
        new Vector3(aabb.max.x, aabb.max.y, aabb.min.z),
        new Vector3(aabb.max.x, aabb.max.y, aabb.max.z),
    };

        bool isOverlapping = false;
        float maxPenetration = 0.0f;

        foreach (var vertex in vertices)
        {
            // Calculate distance of the vertex from the plane
            float distanceToPlane = Vector3.Dot(vertex - planePosition, planeNormal);

            if (distanceToPlane < 0) // Vertex is below the plane
            {
                isOverlapping = true;
                maxPenetration = Mathf.Max(maxPenetration, -distanceToPlane);
            }
        }

        // Handle collision response if overlapping
        if (isOverlapping)
        {
            // Resolve penetration
            Vector3 mtv = planeNormal * maxPenetration;
            objectA.transform.position += mtv;

            // Apply collision response (reflect velocity)
            Vector3 relativeVelocity = objectA.velocity;
            float restitution = 0.8f; // Coefficient of restitution
            objectA.velocity -= (1 + restitution) * Vector3.Dot(relativeVelocity, planeNormal) * planeNormal;

            return true;
        }


        return false;
    }


    public bool CollideSphereHalfspace(FizziksShapeSphere sphere, FizziksShapeHalfspace halfspace, FizikObject objectA)
    {
        Vector3 planeToSphere = sphere.transform.position - halfspace.transform.position;
        float positionAlongNormal = Vector3.Dot(planeToSphere, halfspace.Normal());

        if (positionAlongNormal < sphere.radius)
        {
            Vector3 mtv = halfspace.Normal() * (sphere.radius - positionAlongNormal);
            objectA.transform.position += mtv;
            objectA.velocity = Vector3.zero;
            return true;
        }
        return false;
    }


    public void DestroyFizikObject(FizikObject obj)
    {
        if (objects.Contains(obj))
        {
            objects.Remove(obj);
        }
        Destroy(obj.gameObject);
    }
}
