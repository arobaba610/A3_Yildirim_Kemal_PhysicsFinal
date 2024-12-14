using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class FizziksShape : MonoBehaviour
{
    public enum Shape
    {
        Sphere,
        Plane,
        Halfspace,
        AABB
    }

    public abstract Shape GetShape();
}
