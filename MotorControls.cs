using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotorControls : MonoBehaviour
{
    [SerializeField] private Rigidbody sphereBody; // Rigidbody of the car's main sphere
    
    // Transform points to shoot raycasts off of
    [SerializeField] private Transform castFront, castBack; // Front and back raycast positions
    
    private Rigidbody modelBody; // Rigidbody of the car's model

    [Space]
    [SerializeField] private float modelOffset; // Offset between the model and the sphere
    [SerializeField] private LayerMask groundMask; // Layer mask to define the ground
    [SerializeField] private float alignmentSpeed = 1; // Speed for aligning the car with the ground
    [SerializeField] private float gravityScale = 9.81f; // Gravity strength
    
    [Space]
    [SerializeField] [Min(float.Epsilon)] private float mass = 1; // Mass of the car
    
    [Space]
    [SerializeField] private AnimationCurve tireGripBasedOnVelocity; // Tire grip based on velocity
    [SerializeField] private float maxTurnAngle = 5; // Maximum turning angle
    [SerializeField] private AnimationCurve turnAngleBasedOnVelocity; // Turning angle based on velocity
    [SerializeField] private float leanIntensity = 15; // Intensity of leaning into turns
    
    [Space]
    [SerializeField] private AnimationCurve torqueBasedOnVelocity; // Torque based on velocity
    [SerializeField] private float maxSpeed = 500; // Maximum speed
    [SerializeField] private float dragPercent = 0.1f; // Drag percentage
    
    [Space]
    [SerializeField] private float bobHeight; // Height of bobbing
    [SerializeField] private float bobSpeed; // Speed of bobbing
    private float bobTime = 0; // Time used for bobbing
    public float MaxSpeed { get { return maxSpeed; } } // Accessor for maximum speed

    private float forwardVelocity 
    { 
        get
        {
            // Calculate forward velocity
            float v = Vector3.Dot(currentForward, sphereBody.velocity);
            if (float.IsNaN(v))
            {
                v = 0;
            }
            return v;
        } 
    }
    public float ForwardVelocity { get { return forwardVelocity; } } // Accessor for forward velocity
    public float NormalizedVelocity { get { return Mathf.Clamp01(Mathf.Abs(forwardVelocity) / maxSpeed); } } // Normalized velocity

    private float sideVelocity
    {
        get
        {
            // Calculate side velocity
            float v = Vector3.Dot(modelBody.transform.right, sphereBody.velocity);
            if (float.IsNaN(v))
            {
                v = 0;
            }
            return v;
        }
    }

    private Vector2 inputAxis;

    private Vector3 currentDown;
    private Vector3 currentForward;

    private void Start()
    {
        currentDown = Vector3.down;
        modelBody = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        // Get input axis for steering
        inputAxis = new Vector2(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical"));
        GetCurrentDown();
    }

    private void FixedUpdate()
    {
        HoverModelOverGround();
        RotateModelWithGround();
        Physics.gravity = currentDown * gravityScale;

        Steer();

        Drift();
        Accelerate();
        ApplyDrag();
        Debug.DrawLine(sphereBody.position, sphereBody.position + sphereBody.velocity, Color.cyan);
    }

    private void Drift()
    {
        // Drift the car
        float tireGrip = tireGripBasedOnVelocity.Evaluate(NormalizedVelocity);

        float targetVelocityChange = -sideVelocity * tireGrip;
        float targetAcceleration = targetVelocityChange / Time.fixedDeltaTime;
        sphereBody.AddForce(modelBody.transform.right * (targetAcceleration * mass), ForceMode.Acceleration);
    }

    private void Accelerate()
    {
        // Accelerate the car
        if (forwardVelocity < maxSpeed)
        {
            float targetAcceleration = torqueBasedOnVelocity.Evaluate(NormalizedVelocity) / Time.deltaTime;
            sphereBody.AddForce(transform.forward * (inputAxis.y * targetAcceleration * mass), ForceMode.Acceleration);
        }
    }

    private void Steer()
    {
        // Steer the car
        float rotationAngle = inputAxis.x * (turnAngleBasedOnVelocity.Evaluate(NormalizedVelocity) * maxTurnAngle) * Time.deltaTime;
        Quaternion rotationChange = Quaternion.AngleAxis(rotationAngle, Vector3.up);
        modelBody.MoveRotation(modelBody.rotation * rotationChange);
    }

    /// <summary>
    /// Uses the barycentric coordinate of a raycast to determine an accurate surface normal.
    /// Interpolates between the three normals of a triangle.
    /// </summary>
    private void GetCurrentDown()
    {
        // Get the current surface normal
        RaycastHit hit;
        if (!Physics.Raycast(sphereBody.position, currentDown, out hit, Mathf.Infinity, groundMask))
        {
            currentDown = Vector3.down;
            return;
        }

        // Check for collider and sharedmesh
        MeshCollider hitMeshCollider = hit.collider as MeshCollider;
        if (hitMeshCollider == null)
        {
            Debug.LogError("Hit does not have MeshCollider");
            return;
        }
        if (hitMeshCollider.sharedMesh == null)
        {
            Debug.LogError("Hit does not have sharedMesh");
            return;
        }

        // Get the normals and triangles of the mesh
        Mesh hitMesh = hitMeshCollider.sharedMesh;
        Vector3[] meshNormals = hitMesh.normals;
        int[] meshTris = hitMesh.triangles;

        // Get the normals of the 3 points of the triangle you hit
        int adjustedIndex = hit.triangleIndex * 3;
        Vector3 triNormalX = meshNormals[meshTris[adjustedIndex + 0]];
        Vector3 triNormalY = meshNormals[meshTris[adjustedIndex + 1]];
        Vector3 triNormalZ = meshNormals[meshTris[adjustedIndex + 2]];

        // The point within the triangle that you hit, expressed in a vector 3 where X + Y + Z adds up to 1
        Vector3 baryCenter = hit.barycentricCoordinate;

        // Scale normals with barycenter
        // This makes it so a normal will weigh more when the hit point is closer to it
        triNormalX *= baryCenter.x;
        triNormalY *= baryCenter.y;
        triNormalZ *= baryCenter.z;

        // Interpolate the normals of the 3 points and normalize the result
        Vector3 interpolatedNormal = (triNormalX + triNormalY + triNormalZ).normalized;

        // Convert to worldspace
        Transform hitTransform = hit.collider.transform;
        interpolatedNormal = hitTransform.TransformDirection(interpolatedNormal);

        // Set current down opposite of the interpolated normal
        currentDown = -interpolatedNormal;
    }

    /// <summary>
    /// Aligns rotation with the current up direction and a generated forward direction.
    /// Sends down two raycasts from the front and back and gets a new forward based on those points.
    /// </summary>
    private void RotateModelWithGround()
    {
        currentForward = transform.forward;

        RaycastHit frontHit, backHit;
        if (Physics.Raycast(castFront.position, currentDown, out frontHit, Mathf.Infinity, groundMask) && Physics.Raycast(castBack.position, currentDown, out backHit, Mathf.Infinity, groundMask))
        {
            Vector3 targetForward = (frontHit.point - backHit.point).normalized;
            currentForward = Vector3.Lerp(currentForward, targetForward, alignmentSpeed * Time.deltaTime);
        }
        modelBody.rotation = Quaternion.LookRotation(currentForward, -currentDown);
    }

    private void HoverModelOverGround()
    {
        bobTime += Time.fixedDeltaTime;
        float bobOffset = (1 - NormalizedVelocity) * (Mathf.Sin(bobTime * bobSpeed) * bobHeight);
        Vector3 smoothPos = Vector3.Lerp(transform.position, sphereBody.position + currentDown * (modelOffset + bobOffset), alignmentSpeed * Time.deltaTime);
        modelBody.MovePosition(smoothPos);
    }

    private void LeanBikeIntoTurn()
    {
        float leanAngle = (sideVelocity * NormalizedVelocity) * leanIntensity;
        // Quaternion leanedRotation = Quaternion.AngleAxis(leanAngle, currentForward)
    }

    private void ApplyDrag()
    {
        if (Input.GetAxisRaw("Vertical") == 0)
        {
            float dragForce = (dragPercent * forwardVelocity) / Time.deltaTime;
            sphereBody.AddForce(-transform.forward * (dragForce * mass), ForceMode.Acceleration);
        }
    }
}
