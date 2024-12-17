using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public enum SurfaceMaterial
{
    Steel,
    Wood,
    Rubber,
    Cloth,
    Stone
}

public class FizikObject : MonoBehaviour
{
    public FizziksShape shape = null;
    public Vector3 velocity = Vector3.zero;
    public float drag = 0.1f;
    public float mass = 1f;
    public float gravityScale = 1;
    public bool isStatic = true;

    public float restitution = 0.8f;
    public SurfaceMaterial surfaceMaterial = SurfaceMaterial.Wood; // Default material


    public float staticFrictionCoefficient = 0.5f;
    public float kineticFrictionCoefficient = 0.3f;


    public float launchTime;

    void Start()
    {
        shape = GetComponent<FizziksShape>();
        FizikMotoru.Instance.objects.Add(this);
    }

    private void OnValidate()
    {
        mass = Mathf.Max(0.01f, mass); // Mass
        drag = Mathf.Clamp(drag, 0f, 1f); // Drag coefficient
        gravityScale = Mathf.Max(0f, gravityScale); // Gravity scale
    }
}



