using UnityEngine;
public class FizziksShapePlane : FizziksShape
{

    public enum Type
    {
        Plane,
        halfspace
    }
    public Type PlaneType = Type.Plane;
    public override Shape GetShape()
    {
        return Shape.Plane;
    }

    public Vector3 GetPosition()
    {
        return transform.position;
    }

    public Vector3 Normal()
    {
        return transform.up; 
    }
}
