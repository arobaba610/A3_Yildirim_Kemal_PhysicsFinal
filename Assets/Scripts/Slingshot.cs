using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Slingshot : MonoBehaviour
{
    public Transform slingshotAnchor; 
    public Transform birdPrefabSphere; 
    public Transform birdPrefabAABB; 

    private Transform currentBird; 
    private bool isDragging = false; 
    private Vector3 launchDirection; 

    [SerializeField]
    private float maxStretch = 5.0f; 

    [SerializeField]
    private float launchForceMultiplier = 10.0f;

    private bool useSphereBird = true; 

    private FizikMotoru physicsEngine; 

    void Start()
    {
        physicsEngine = FizikMotoru.Instance;
        SpawnBird();
    }

    void Update()
    {
        HandleInput();
    }

    private void HandleInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            StartDragging();
        }
        else if (Input.GetMouseButton(0) && isDragging)
        {
            Drag();
        }
        else if (Input.GetMouseButtonUp(0) && isDragging)
        {
            LaunchBird();
        }

        if (Input.GetKeyDown(KeyCode.Q))
        {
            SwitchBirdType();
        }
    }

    private void StartDragging()
    {
        isDragging = true;
    }

    private void Drag()
    {
        Vector3 mouseWorldPos = GetMouseWorldPosition();

        // Calculate drag vector
        Vector3 dragVector = mouseWorldPos - slingshotAnchor.position;

        // Clamp to max stretch distance
        if (dragVector.magnitude > maxStretch)
        {
            dragVector = dragVector.normalized * maxStretch;
        }

        
        launchDirection = -dragVector; 
        currentBird.position = slingshotAnchor.position + dragVector;
    }

    private void LaunchBird()
    {
        isDragging = false;

        
        float speed = launchDirection.magnitude * launchForceMultiplier;
        Vector3 launchVelocity = launchDirection.normalized * speed;

        // Apply launch velocity to the physics objects
        FizikObject birdObject = currentBird.GetComponent<FizikObject>();
        birdObject.velocity = launchVelocity;
        birdObject.isStatic = false;
        birdObject.launchTime = Time.time;

        Debug.Log($"Launching Bird: Velocity={birdObject.velocity}, Force={launchDirection.magnitude}");

        
       
    }

    private void SpawnBird()
    {
        if (currentBird != null)
        {
            physicsEngine.DestroyFizikObject(currentBird.GetComponent<FizikObject>());
        }

        if (useSphereBird)
        {
            SpawnSphereBird();
        }
        else
        {
            SpawnAABBBird();
        }
    }

    private void SpawnSphereBird()
    {
        currentBird = Instantiate(birdPrefabSphere, slingshotAnchor.position, Quaternion.identity);
        FizziksShapeSphere sphereShape = currentBird.GetComponent<FizziksShapeSphere>();
        sphereShape.radius = 0.5f;

        FizikObject birdObject = currentBird.GetComponent<FizikObject>();
        birdObject.shape = sphereShape;
        birdObject.mass = 1.0f;
        birdObject.isStatic = true; // Set to static until launched
    }

    private void SpawnAABBBird()
    {
        currentBird = Instantiate(birdPrefabAABB, slingshotAnchor.position, Quaternion.identity);

        FizziksShapeAABB aabbShape = currentBird.GetComponent<FizziksShapeAABB>();
        aabbShape.RecalculateBounds();

        FizikObject birdObject = currentBird.GetComponent<FizikObject>();
        birdObject.shape = aabbShape;
        birdObject.mass = 1.5f;
        birdObject.isStatic = true; // Set to static until launched
    }

    private void SwitchBirdType()
    {
        useSphereBird = !useSphereBird;
        SpawnBird();
    }

    private Vector3 GetMouseWorldPosition()
    {
        Vector3 mouseScreenPos = Input.mousePosition;
        mouseScreenPos.z = Camera.main.WorldToScreenPoint(slingshotAnchor.position).z; // Maintain depth for 3D projection
        Vector3 mouseWorldPos = Camera.main.ScreenToWorldPoint(mouseScreenPos);

        return new Vector3(mouseWorldPos.x, mouseWorldPos.y, slingshotAnchor.position.z); // Lock to 2D plane
    }
}
